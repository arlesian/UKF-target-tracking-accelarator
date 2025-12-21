import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv, cholesky
import os

## notes to myself tomorrow:
# - process noise covariance Q should be small since drone dynamics are smooth
# - measurement noise covariance R should be larger to reflect GPS inaccuracies
# - study UKF theory to understand update and predict steps better!!

## state transition of the drone
import numpy as np
np.random.seed(4)  # For reproducibility

def true_trajectory(t):
    base_x = 50 * np.sin(0.3 * t)
    base_y = 30 * np.sin(0.7 * t + 1)
    base_z = 20 * np.cos(0.5 * t)
    
    # Add erratic noise (scale as needed)
    noise_x = np.random.normal(0, 5)  # Mean 0, std dev 5
    noise_y = np.random.normal(0, 3)
    noise_z = np.random.normal(0, 2)
    
    x = base_x + noise_x
    y = base_y + noise_y
    z = base_z + noise_z
    return np.array([x, y, z])

class UKF:
    def __init__(self, x_dim, z_dim, alpha=1e-3, beta=2, kappa=0):
        self.x = np.zeros(x_dim)
        self.P = np.eye(x_dim)

        self.Q = np.eye(x_dim)  # Process noise covariance
        self.R = np.eye(z_dim)  # Measurement noise covariance

        self.lambda_ = None
        self.wm = None
        self.wc = None
        self._compute_weights(x_dim, alpha, beta, kappa)

    def _compute_weights(self, n, alpha, beta, kappa):
        self.lambda_ = alpha**2 * (n + kappa) - n
        self.wm = np.full(2*n+1, 1/(2*(n+self.lambda_)))
        self.wc = np.full(2*n+1, 1/(2*(n+self.lambda_)))
        self.wm[0] = self.lambda_/(n+self.lambda_)
        self.wc[0] = self.lambda_/(n+self.lambda_) + (1 - alpha**2 + beta)

    def _sigma_points(self, x, P):
        n = len(x)
        sigma_points = np.zeros((2*n + 1, n))
        sigma_points[0] = x
        sqrt_P = cholesky((n + self.lambda_) * P)

        for i in range(n):
            sigma_points[i + 1] = x + sqrt_P[:, i]
            sigma_points[n + i + 1] = x - sqrt_P[:, i]

        return sigma_points

    ## state transition model of the UKF
    def f(self, x, dt):
        px, py, pz, vx, vy, vz = x
        return np.array([
            px+vx*dt,
            py+vy*dt,
            pz+vz*dt,
            vx,
            vy,
            vz
        ])

    def _propagate(self, sigma_points, dt):
        return np.array([self.f(sp, dt) for sp in sigma_points])

    ## calculates the priori mean and covariance
    def predict(self, dt):
        sigma_points = self._sigma_points(self.x, self.P)
        propagated_sigma_points = self._propagate(sigma_points, dt)

        self.x = np.sum(self.wm[:, None] * propagated_sigma_points, axis=0)

        self.P = self.Q.copy()
        for i in range(propagated_sigma_points.shape[0]):
            diff = propagated_sigma_points[i] - self.x
            self.P += self.wc[i] * np.outer(diff, diff)

        self.propagated_sigma_points = propagated_sigma_points


    ## measurement model of the UKF
    def h(self, x):
        px, py, pz, vx, vy, vz = x
        return np.array([px, py, pz])  # Measuring position only

    def _measure(self, sigma_points):
        return np.array([self.h(sp) for sp in sigma_points])

    ## calculates the posteriori mean and covariance
    def update(self, z):
        sigma_points = self.propagated_sigma_points
        measured_sigma_points = self._measure(sigma_points)

        z_pred = np.sum(self.wm[:, None] * measured_sigma_points, axis=0)

        S = self.R.copy()
        for i in range(measured_sigma_points.shape[0]):
            diff = measured_sigma_points[i] - z_pred
            S += self.wc[i] * np.outer(diff, diff)

        cross_cov = np.zeros((self.x.shape[0], z.shape[0]))
        for i in range(sigma_points.shape[0]):
            state_diff = sigma_points[i] - self.x
            meas_diff = measured_sigma_points[i] - z_pred
            cross_cov += self.wc[i] * np.outer(state_diff, meas_diff)

        K = cross_cov @ inv(S)

        self.x += K @ (z - z_pred)
        self.P -= K @ S @ K.T

def random_diag(n, low=0.01, high=1.0, seed=None):
    """Return an n x n diagonal matrix with random values sampled
    uniformly from [low, high] on the diagonal.
    """
    if seed is not None:
        np.random.seed(seed)
    vals = np.random.uniform(low, high, size=n)
    return np.diag(vals)

def simulate_ukf():
    dt = 0.1
    time_steps = 300
    ukf = UKF(x_dim=6, z_dim=3)
    # Use random diagonal process and measurement noise covariances
    ukf.Q = random_diag(6, low=0.01, high=0.1) 
    ukf.R = random_diag(3, low=0.1, high=1.0)
    true_states = []
    measurements = []
    estimates = []
    for t in range(time_steps):
        print(f"Time step {t+1}/{time_steps}", end='\r')
        true_state = true_trajectory(t*dt)
        true_states.append(true_state)

        measurement = true_state + np.random.multivariate_normal(np.zeros(3), ukf.R)
        measurements.append(measurement)

        ukf.predict(dt)
        ukf.update(measurement)
        estimates.append(ukf.x[:3])

    true_states = np.array(true_states)
    measurements = np.array(measurements)
    estimates = np.array(estimates)
    estimation_errors = np.linalg.norm(true_states - estimates, axis=1)
    print(f"\nMean Estimation Error: {np.mean(estimation_errors):.2f}")

    ## Plotting estimation error over time
    plt.figure(figsize=(10, 5))
    plt.plot(estimation_errors, label='Estimation Error')
    plt.xlabel('Time Step')
    plt.ylabel('Error Magnitude')
    plt.title('UKF Estimation Error Over Time')
    plt.legend()
    plt.grid()
    os.path.exists("ukf_estimation_error.png") and os.remove("ukf_estimation_error.png") ## delete existing file
    plt.savefig("ukf_estimation_error.png")

    ## Plotting trajectories
    plt.figure(figsize=(12, 8))
    plt.plot(true_states[:, 0], true_states[:, 1], label='True Trajectory', color='g')
    plt.scatter(measurements[:, 0], measurements[:, 1], label='Measurements', color='r', s=10)
    plt.plot(estimates[:, 0], estimates[:, 1], label='UKF Estimate', color='b')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('UKF Drone Trajectory Estimation')
    plt.legend()
    plt.grid()
    os.path.exists("ukf_drone_trajectory.png") and os.remove("ukf_drone_trajectory.png") ## delete existing file
    plt.savefig("ukf_drone_trajectory.png")

if __name__ == "__main__":
    print("Starting UKF simulation...")
    simulate_ukf()