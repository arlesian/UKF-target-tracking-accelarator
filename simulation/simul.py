import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv, cholesky
import os

## notes to myself:
# FIX: fix errors regarding cova not being pos def -> fixed by adding small value to diag
# FIX: ensured angles are wrapped correctly during mean and diff calculations
# FIX: circular mean for angles in measurement prediction
# FIX: modified posterori calculation using joint measurement update
# NEED: find the q-format to use for verilog implementation

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
    px = 5 * np.sin(0.05 * t)
    py = 3 * np.cos(0.05 * t)
    pz = 0.0

    vx = 5 * 0.05 * np.cos(0.05 * t)
    vy = -3 * 0.05 * np.sin(0.05 * t)
    vz = 0.0

    return np.array([px, py, pz, vx, vy, vz])


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

        # Radar: [r, az, el, r_dot]
        self.R_radar = np.diag([5.0, 0.01, 0.01, 5.0])

        # IR: [az, el]
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
        # FIX: ensure P is positive definite
        sqrtP = cholesky((n + self.lambda_) * (P + 1e-5 * np.eye(P.shape[0])))
        for i in range(n):
            sigma[i+1] = x + sqrtP[:, i]
            sigma[n+i+1] = x - sqrtP[:, i]
        return sigma

    def _wrap_angle_diff(self, d, idxs):
        for j in idxs:
            d[j] = (d[j] + np.pi) % (2*np.pi) - np.pi
        return d

    def _circular_mean(self, angles):
        s = np.sum(self.wm * np.sin(angles))
        c = np.sum(self.wm * np.cos(angles))
        return np.arctan2(s, c)

    # -----------------------------------------------------
    # Motion model
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

        ### FIX: add Q to covariance
        self.P = self.Q.copy()
        for i in range(sigma_f.shape[0]):
            d = sigma_f[i] - self.x
            self.P += self.wc[i] * np.outer(d, d)

        self.P = 0.5 * (self.P + self.P.T)

    # -----------------------------------------------------
    # Measurement models
    # -----------------------------------------------------
    def h_radar(self, x, ego):
        px, py, pz, vx, vy, vz = x
        ego_pos = ego[:3]
        ego_vel = ego[3:]

        dx, dy, dz = px-ego_pos[0], py-ego_pos[1], pz-ego_pos[2]
        rvx, rvy, rvz = vx-ego_vel[0], vy-ego_vel[1], vz-ego_vel[2]

        return cartesian_to_spherical(dx, dy, dz, rvx, rvy, rvz)

    def h_ir(self, x, ego):
        px, py, pz, *_ = x
        dx, dy, dz = px-ego[0], py-ego[1], pz-ego[2]
        return cartesian_to_ir(dx, dy, dz)

    # -----------------------------------------------------
    # Joint update
    # -----------------------------------------------------
    def update(self, z_radar, z_ir, ego):

        sigma = self._sigma_points(self.x, self.P)
        sigma_f = np.array([self.f(sp, 0) for sp in sigma])

        Z_radar = np.array([self.h_radar(sp, ego) for sp in sigma_f])
        Z_ir = np.array([self.h_ir(sp, ego) for sp in sigma_f])

        Z_joint = np.hstack((Z_radar, Z_ir))
        z_joint = np.hstack((z_radar, z_ir))

        z_pred = np.zeros_like(z_joint)

        ### FIX: circular mean for angles
        z_pred[0] = np.sum(self.wm * Z_joint[:, 0])     # range
        z_pred[3] = np.sum(self.wm * Z_joint[:, 3])     # r_dot
        for idx in [1, 2, 4, 5]:
            z_pred[idx] = self._circular_mean(Z_joint[:, idx])

        dim_z = z_joint.shape[0]
        S = np.zeros((dim_z, dim_z))

        ### FIX: add measurement noise
        R_joint = np.block([
            [self.R_radar, np.zeros((4, 2))],
            [np.zeros((2, 4)), self.R_ir]
        ])
        S += R_joint

        ang_idx = [1, 2, 4, 5]
        for i in range(Z_joint.shape[0]):
            dz = Z_joint[i] - z_pred
            dz = self._wrap_angle_diff(dz, ang_idx)
            S += self.wc[i] * np.outer(dz, dz)

        Pxz = np.zeros((self.x.shape[0], dim_z))
        for i in range(Z_joint.shape[0]):
            dx = sigma_f[i] - self.x
            dz = Z_joint[i] - z_pred
            dz = self._wrap_angle_diff(dz, ang_idx)
            Pxz += self.wc[i] * np.outer(dx, dz)

        S = 0.5 * (S + S.T) + 1e-6 * np.eye(dim_z)
        K = Pxz @ inv(S)

        y = z_joint - z_pred
        y = self._wrap_angle_diff(y, ang_idx)

        self.x += K @ y
        self.P -= K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)


# =========================================================
# Simulation
# =========================================================
def simulate(a, b, k):
    dt = 0.1
    steps = 300
    ukf = UKF(6, alpha=a, beta=b, kappa=k)

    true_pos, est_pos = [], []
    prev_range = None

    for k in range(steps):
        print(f"Step {k+1}/{steps}", end="\r")
        t = k * dt
        ego = ego_trajectory(t)
        tgt = true_trajectory(t)

        rel = tgt - ego[:3]
        r = np.linalg.norm(rel) + 1e-6

        if prev_range is None:
            r_dot = 0.0
        else:
            r_dot = (r - prev_range) / dt
        prev_range = r

        v_rel = r_dot * rel / r

        z_radar = cartesian_to_spherical(*rel, *v_rel) + \
                  np.random.multivariate_normal(np.zeros(4), ukf.R_radar)

        z_ir = cartesian_to_ir(*rel) + \
               np.random.multivariate_normal(np.zeros(2), ukf.R_ir)

        ukf.predict(dt)
        ukf.update(z_radar, z_ir, ego)

        true_pos.append(tgt)
        est_pos.append(ukf.x[:3])

    true_pos = np.array(true_pos)
    est_pos = np.array(est_pos)

    errors = np.linalg.norm(true_pos - est_pos, axis=1)
    mean_error = np.mean(errors)

    plt.figure(figsize=(10, 6))
    plt.plot(true_pos[:, 0], true_pos[:, 1], label="True")
    plt.plot(est_pos[:, 0], est_pos[:, 1], label="UKF")
    plt.legend()
    plt.grid()
    plt.savefig("ukf_radar_ir.png")
    plt.show()

    return mean_error


if __name__ == "__main__":

    # running with best params from grid search
    alpha = 0.2
    beta = 2
    kappa = 3

    mean_err = simulate(alpha, beta, kappa)
    print(f"Mean position error: {mean_err:.3f}")