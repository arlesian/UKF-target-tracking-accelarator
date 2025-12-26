import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv, cholesky
import os

## notes to myself:
# fix errors regarding cova not being pos def
# review spherical to cartesian conversion logic
# find the q-format to use for verilog implementation

np.random.seed(4)

# =========================================================
# True target trajectory
# =========================================================
def true_trajectory(t):
    base_x = 50 * np.sin(0.3 * t)
    base_y = 30 * np.sin(0.7 * t + 1)
    base_z = 20 * np.cos(0.5 * t)

    noise = np.array([
        np.random.normal(0, 5),
        np.random.normal(0, 3),
        np.random.normal(0, 2)
    ])
    return np.array([base_x, base_y, base_z]) + noise


# =========================================================
# Ego (IMU-based) trajectory
# =========================================================
def ego_trajectory(t):
    return np.array([
        5 * np.sin(0.05 * t),
        3 * np.cos(0.05 * t),
        0.0
    ])


# =========================================================
# Coordinate transforms
# =========================================================
def cartesian_to_spherical(dx, dy, dz, vx, vy, vz):
    r = np.sqrt(dx**2 + dy**2 + dz**2) + 1e-6
    az = np.arctan2(dy, dx)
    el = np.arctan2(dz, np.sqrt(dx**2 + dy**2))
    r_dot = (dx*vx + dy*vy + dz*vz) / r
    return np.array([r, az, el, r_dot])


def cartesian_to_ir(dx, dy, dz):
    az = np.arctan2(dy, dx)
    el = np.arctan2(dz, np.sqrt(dx**2 + dy**2))
    return np.array([az, el])


# =========================================================
# Unscented Kalman Filter
# =========================================================
class UKF:
    def __init__(self, x_dim, alpha=1e-3, beta=2, kappa=0):
        self.x = np.zeros(x_dim)
        self.P = np.eye(x_dim)

        self.Q = np.eye(x_dim) * 0.05

        self.R_radar = np.diag([5.0, 0.01, 0.01, 1.0])
        self.R_ir = np.diag([0.01, 0.01])

        self._compute_weights(x_dim, alpha, beta, kappa)

    def _compute_weights(self, n, alpha, beta, kappa):
        self.lambda_ = alpha**2 * (n + kappa) - n
        self.wm = np.full(2*n+1, 1/(2*(n+self.lambda_)))
        self.wc = np.full(2*n+1, 1/(2*(n+self.lambda_)))
        self.wm[0] = self.lambda_ / (n + self.lambda_)
        self.wc[0] = self.lambda_ / (n + self.lambda_) + (1 - alpha**2 + beta)

    def _sigma_points(self, x, P):
        n = len(x)
        sigma = np.zeros((2*n+1, n))
        sigma[0] = x
        sqrtP = cholesky((n + self.lambda_) * P)
        for i in range(n):
            sigma[i+1] = x + sqrtP[:, i]
            sigma[n+i+1] = x - sqrtP[:, i]
        return sigma

    # -----------------------------------------------------
    # Motion model (IMU input already applied externally)
    # -----------------------------------------------------
    def f(self, x, dt):
        px, py, pz, vx, vy, vz = x
        return np.array([
            px + vx*dt,
            py + vy*dt,
            pz + vz*dt,
            vx,
            vy,
            vz
        ])

    def predict(self, dt):
        sigma = self._sigma_points(self.x, self.P)
        sigma_f = np.array([self.f(sp, dt) for sp in sigma])

        self.x = np.sum(self.wm[:, None] * sigma_f, axis=0)

        self.P = self.Q.copy()
        for i in range(sigma_f.shape[0]):
            d = sigma_f[i] - self.x
            self.P += self.wc[i] * np.outer(d, d)

        self.sigma_f = sigma_f

    # -----------------------------------------------------
    # Measurement models
    # -----------------------------------------------------
    def h_radar(self, x, ego):
        px, py, pz, vx, vy, vz = x
        dx, dy, dz = px-ego[0], py-ego[1], pz-ego[2]
        return cartesian_to_spherical(dx, dy, dz, vx, vy, vz)

    def h_ir(self, x, ego):
        px, py, pz, *_ = x
        dx, dy, dz = px-ego[0], py-ego[1], pz-ego[2]
        return cartesian_to_ir(dx, dy, dz)

    # -----------------------------------------------------
    # Update step (sequential sensor fusion)
    # -----------------------------------------------------
    def update(self, z_radar, z_ir, ego):
        # ---------- Radar ----------
        Z = np.array([self.h_radar(sp, ego) for sp in self.sigma_f])
        z_pred = np.sum(self.wm[:, None] * Z, axis=0)

        S = self.R_radar.copy()
        for i in range(Z.shape[0]):
            d = Z[i] - z_pred
            S += self.wc[i] * np.outer(d, d)

        Pxz = np.zeros((self.x.shape[0], z_radar.shape[0]))
        for i in range(Z.shape[0]):
            Pxz += self.wc[i] * np.outer(self.sigma_f[i] - self.x, Z[i] - z_pred)

        K = Pxz @ inv(S)
        self.x += K @ (z_radar - z_pred)
        self.P -= K @ S @ K.T

        # ---------- IR ----------
        Z = np.array([self.h_ir(sp, ego) for sp in self.sigma_f])
        z_pred = np.sum(self.wm[:, None] * Z, axis=0)

        S = self.R_ir.copy()
        for i in range(Z.shape[0]):
            d = Z[i] - z_pred
            S += self.wc[i] * np.outer(d, d)

        Pxz = np.zeros((self.x.shape[0], z_ir.shape[0]))
        for i in range(Z.shape[0]):
            Pxz += self.wc[i] * np.outer(self.sigma_f[i] - self.x, Z[i] - z_pred)

        K = Pxz @ inv(S)
        self.x += K @ (z_ir - z_pred)
        self.P -= K @ S @ K.T


# =========================================================
# Simulation
# =========================================================
def simulate():
    dt = 0.1
    steps = 300
    ukf = UKF(6)

    true_pos, est_pos = [], []

    for k in range(steps):
        t = k * dt
        ego = ego_trajectory(t)
        tgt = true_trajectory(t)

        rel = tgt - ego

        z_radar = cartesian_to_spherical(*rel, 0, 0, 0) + \
                  np.random.multivariate_normal(np.zeros(4), ukf.R_radar)

        z_ir = cartesian_to_ir(*rel) + \
               np.random.multivariate_normal(np.zeros(2), ukf.R_ir)

        ukf.predict(dt)
        ukf.update(z_radar, z_ir, ego)

        true_pos.append(tgt)
        est_pos.append(ukf.x[:3])

    true_pos = np.array(true_pos)
    est_pos = np.array(est_pos)

    plt.figure(figsize=(10, 6))
    plt.plot(true_pos[:, 0], true_pos[:, 1], label="True")
    plt.plot(est_pos[:, 0], est_pos[:, 1], label="UKF")
    plt.legend()
    plt.grid()
    plt.title("Radar + IR UKF Tracking with Ego Motion")
    plt.savefig("ukf_radar_ir.png")
    plt.show()


if __name__ == "__main__":
    simulate()
