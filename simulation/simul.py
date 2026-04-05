import os
import numpy as np
from numpy.linalg import cholesky, solve


def true_state(t):
    px = 50.0 * np.sin(0.1 * t)
    py = 30.0 * np.sin(0.07 * t + 1.0)
    pz = 20.0 * np.cos(0.05 * t)

    vx = 5.0 * np.cos(0.1 * t)
    vy = 2.1 * np.cos(0.07 * t + 1.0)
    vz = -1.0 * np.sin(0.05 * t)
    return np.array([px, py, pz, vx, vy, vz], dtype=float)


def process_noise_covariance(dt, accel_std):
    q = accel_std ** 2
    block = np.array(
        [
            [dt**4 / 4.0, dt**3 / 2.0],
            [dt**3 / 2.0, dt**2],
        ],
        dtype=float,
    )

    cov = np.zeros((6, 6), dtype=float)
    for idx in range(3):
        start = 2 * idx
        cov[start:start + 2, start:start + 2] = q * block

    # Reorder from [x, vx, y, vy, z, vz] blocks to [x, y, z, vx, vy, vz].
    order = [0, 2, 4, 1, 3, 5]
    return cov[np.ix_(order, order)]


class UKF:
    def __init__(self, x_dim, z_dim, alpha=0.3, beta=2.0, kappa=0.0):
        self.x = np.zeros(x_dim, dtype=float)
        self.P = np.eye(x_dim, dtype=float)

        self.Q = np.eye(x_dim, dtype=float) * 1e-3
        self.R = np.eye(z_dim, dtype=float)

        self.lambda_ = None
        self.wm = None
        self.wc = None
        self._compute_weights(x_dim, alpha, beta, kappa)

    def _compute_weights(self, n, alpha, beta, kappa):
        self.lambda_ = alpha**2 * (n + kappa) - n
        scale = n + self.lambda_
        if scale <= 0.0:
            raise ValueError(
                "Invalid UKF parameters: alpha and kappa must produce n + lambda > 0."
            )
        self.wm = np.full(2 * n + 1, 1.0 / (2.0 * scale), dtype=float)
        self.wc = np.full(2 * n + 1, 1.0 / (2.0 * scale), dtype=float)
        self.wm[0] = self.lambda_ / scale
        self.wc[0] = self.lambda_ / scale + (1.0 - alpha**2 + beta)

    def _stabilize_covariance(self, cov, min_diagonal=1e-9):
        cov = 0.5 * (cov + cov.T)
        diagonal = np.diag(cov)
        min_value = np.min(diagonal)
        if min_value < min_diagonal:
            cov = cov + np.eye(cov.shape[0], dtype=float) * (min_diagonal - min_value)
        return cov

    def _sigma_points(self, x, P):
        n = len(x)
        sigma_points = np.zeros((2 * n + 1, n), dtype=float)
        sigma_points[0] = x

        stabilized_P = self._stabilize_covariance(P)
        sqrt_P = cholesky((n + self.lambda_) * stabilized_P)
        for i in range(n):
            sigma_points[i + 1] = x + sqrt_P[:, i]
            sigma_points[n + i + 1] = x - sqrt_P[:, i]

        return sigma_points

    def f(self, x, dt):
        px, py, pz, vx, vy, vz = x
        return np.array(
            [
                px + vx * dt,
                py + vy * dt,
                pz + vz * dt,
                vx,
                vy,
                vz,
            ],
            dtype=float,
        )

    def h(self, x):
        return x[:3].copy()

    def predict(self, dt):
        sigma_points = self._sigma_points(self.x, self.P)
        propagated = np.array([self.f(point, dt) for point in sigma_points], dtype=float)

        x_pred = np.sum(self.wm[:, None] * propagated, axis=0)

        P_pred = self.Q.copy()
        for i in range(propagated.shape[0]):
            diff = propagated[i] - x_pred
            P_pred += self.wc[i] * np.outer(diff, diff)

        self.x = x_pred
        self.P = self._stabilize_covariance(P_pred)

    def update(self, z):
        sigma_points = self._sigma_points(self.x, self.P)
        measured = np.array([self.h(point) for point in sigma_points], dtype=float)

        z_pred = np.sum(self.wm[:, None] * measured, axis=0)

        S = self.R.copy()
        cross_cov = np.zeros((self.x.shape[0], z.shape[0]), dtype=float)
        for i in range(measured.shape[0]):
            dz = measured[i] - z_pred
            dx = sigma_points[i] - self.x
            S += self.wc[i] * np.outer(dz, dz)
            cross_cov += self.wc[i] * np.outer(dx, dz)

        K = solve(S.T, cross_cov.T).T
        innovation = z - z_pred

        self.x = self.x + K @ innovation
        self.P = self._stabilize_covariance(self.P - K @ S @ K.T)


def simulate_ukf(seed=7, time_steps=300, dt=0.1):
    if time_steps < 2:
        raise ValueError("time_steps must be at least 2 to initialize velocity.")
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    rng = np.random.default_rng(seed)

    ukf = UKF(x_dim=6, z_dim=3)
    ukf.Q = process_noise_covariance(dt, accel_std=0.35)
    ukf.R = np.diag([2.0**2, 2.0**2, 1.5**2]).astype(float)

    true_states = np.array([true_state(k * dt) for k in range(time_steps)], dtype=float)
    true_positions = true_states[:, :3]
    measurements = true_positions + rng.multivariate_normal(
        mean=np.zeros(3, dtype=float),
        cov=ukf.R,
        size=time_steps,
    )

    ukf.x[:3] = measurements[0]
    ukf.x[3:] = (measurements[1] - measurements[0]) / dt
    ukf.P = np.diag([10.0, 10.0, 10.0, 25.0, 25.0, 25.0]).astype(float)

    estimates = np.zeros((time_steps, 6), dtype=float)
    estimates[0] = ukf.x

    for k in range(1, time_steps):
        ukf.predict(dt)
        ukf.update(measurements[k])
        estimates[k] = ukf.x

    position_errors = true_positions - estimates[:, :3]
    rmse = np.sqrt(np.mean(np.sum(position_errors**2, axis=1)))

    plot_saved = False
    plot_path = os.path.join("simulation", "ukf_simulation.png")
    try:
        os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(true_positions[:, 0], true_positions[:, 1], true_positions[:, 2], label="True trajectory", color="g")
        ax.scatter(measurements[:, 0], measurements[:, 1], measurements[:, 2], label="Measurements", color="r", s=10, alpha=0.5)
        ax.plot(estimates[:, 0], estimates[:, 1], estimates[:, 2], label="UKF estimate", color="b")
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.set_title(f"UKF Synthetic Tracking (RMSE={rmse:.2f})")
        ax.legend()
        ax.grid(True)
        fig.tight_layout()
        os.makedirs(os.path.dirname(plot_path), exist_ok=True)
        fig.savefig(plot_path, dpi=160)
        plt.close(fig)
        plot_saved = True
    except ModuleNotFoundError:
        pass

    return {
        "rmse": rmse,
        "true_states": true_states,
        "measurements": measurements,
        "estimates": estimates,
        "plot_saved": plot_saved,
        "plot_path": plot_path,
    }


if __name__ == "__main__":
    result = simulate_ukf()
    print(f"Finished UKF simulation. Position RMSE: {result['rmse']:.3f}")
    if result["plot_saved"]:
        print(f"Saved plot to {result['plot_path']}")
    else:
        print("matplotlib not installed; skipped plot generation")
