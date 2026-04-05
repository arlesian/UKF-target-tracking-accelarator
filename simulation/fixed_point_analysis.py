import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from simul import UKF, process_noise_covariance, true_state


def update_stats(stats, name, value):
    data = np.asarray(value, dtype=float)
    abs_max = float(np.max(np.abs(data)))
    current = stats.get(name)
    if current is None:
        stats[name] = {
            "min": float(np.min(data)),
            "max": float(np.max(data)),
            "abs_max": abs_max,
        }
        return

    current["min"] = min(current["min"], float(np.min(data)))
    current["max"] = max(current["max"], float(np.max(data)))
    current["abs_max"] = max(current["abs_max"], abs_max)


def required_signed_integer_bits(max_abs, guard_bits):
    if max_abs <= 0.0:
        magnitude_bits = 0
    else:
        magnitude_bits = max(0, math.ceil(math.log2(max_abs)))
    return 1 + magnitude_bits + guard_bits


def recommend_format(max_abs, total_bits, guard_bits):
    integer_bits = required_signed_integer_bits(max_abs, guard_bits)
    fractional_bits = total_bits - integer_bits
    if fractional_bits < 0:
        return {
            "fits": False,
            "integer_bits": integer_bits,
            "fractional_bits": fractional_bits,
            "range_limit": None,
            "resolution": None,
        }

    return {
        "fits": True,
        "integer_bits": integer_bits,
        "fractional_bits": fractional_bits,
        "range_limit": 2 ** (integer_bits - 1),
        "resolution": 2.0 ** (-fractional_bits),
    }


def run_single_trial(seed, time_steps, dt, accel_std, measurement_std):
    rng = np.random.default_rng(seed)

    ukf = UKF(x_dim=6, z_dim=3)
    ukf.Q = process_noise_covariance(dt, accel_std=accel_std)
    ukf.R = np.diag(np.square(measurement_std)).astype(float)

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

    stats = {}
    update_stats(stats, "true_state", true_states)
    update_stats(stats, "measurement", measurements)
    update_stats(stats, "process_noise_covariance_Q", ukf.Q)
    update_stats(stats, "measurement_noise_covariance_R", ukf.R)
    update_stats(stats, "initial_state", ukf.x)
    update_stats(stats, "initial_covariance_P", ukf.P)

    for k in range(1, time_steps):
        sigma_points = ukf._sigma_points(ukf.x, ukf.P)
        propagated = np.array([ukf.f(point, dt) for point in sigma_points], dtype=float)
        x_pred = np.sum(ukf.wm[:, None] * propagated, axis=0)

        P_pred = ukf.Q.copy()
        for i in range(propagated.shape[0]):
            diff = propagated[i] - x_pred
            P_pred += ukf.wc[i] * np.outer(diff, diff)
        P_pred = ukf._stabilize_covariance(P_pred)

        sigma_points_pred = ukf._sigma_points(x_pred, P_pred)
        measured = np.array([ukf.h(point) for point in sigma_points_pred], dtype=float)
        z_pred = np.sum(ukf.wm[:, None] * measured, axis=0)

        S = ukf.R.copy()
        cross_cov = np.zeros((ukf.x.shape[0], measurements[k].shape[0]), dtype=float)
        for i in range(measured.shape[0]):
            dz = measured[i] - z_pred
            dx = sigma_points_pred[i] - x_pred
            S += ukf.wc[i] * np.outer(dz, dz)
            cross_cov += ukf.wc[i] * np.outer(dx, dz)

        K = np.linalg.solve(S.T, cross_cov.T).T
        innovation = measurements[k] - z_pred
        x_upd = x_pred + K @ innovation
        P_upd = ukf._stabilize_covariance(P_pred - K @ S @ K.T)

        update_stats(stats, "state_sigma_points", sigma_points)
        update_stats(stats, "propagated_sigma_points", propagated)
        update_stats(stats, "predicted_state", x_pred)
        update_stats(stats, "predicted_covariance_P", P_pred)
        update_stats(stats, "measurement_sigma_points", measured)
        update_stats(stats, "predicted_measurement", z_pred)
        update_stats(stats, "innovation", innovation)
        update_stats(stats, "innovation_covariance_S", S)
        update_stats(stats, "cross_covariance_Pxz", cross_cov)
        update_stats(stats, "kalman_gain_K", K)
        update_stats(stats, "updated_state", x_upd)
        update_stats(stats, "updated_covariance_P", P_upd)

        ukf.x = x_upd
        ukf.P = P_upd

    return stats


def merge_stats(all_stats, trial_stats):
    for name, values in trial_stats.items():
        if name not in all_stats:
            all_stats[name] = values.copy()
            continue
        all_stats[name]["min"] = min(all_stats[name]["min"], values["min"])
        all_stats[name]["max"] = max(all_stats[name]["max"], values["max"])
        all_stats[name]["abs_max"] = max(all_stats[name]["abs_max"], values["abs_max"])


def analyze_ranges(trials, time_steps, dt, accel_std, measurement_std):
    merged = {}
    for seed in range(trials):
        merge_stats(
            merged,
            run_single_trial(
                seed=seed,
                time_steps=time_steps,
                dt=dt,
                accel_std=accel_std,
                measurement_std=measurement_std,
            ),
        )
    return merged


def print_report(stats, total_bits, guard_bits):
    print("Fixed-point range analysis for UKF datapath")
    print("Q format below is reported as signed QI.F, where I includes the sign bit.")
    print(f"Guard integer bits added: {guard_bits}")
    print()

    header = (
        f"{'signal':<32} {'min':>12} {'max':>12} {'|x|_max':>12} "
        + " ".join(f"{bits:>18}b" for bits in total_bits)
    )
    print(header)
    print("-" * len(header))

    for name in sorted(stats):
        row = (
            f"{name:<32} "
            f"{stats[name]['min']:>12.5f} "
            f"{stats[name]['max']:>12.5f} "
            f"{stats[name]['abs_max']:>12.5f}"
        )

        for bits in total_bits:
            fmt = recommend_format(stats[name]["abs_max"], bits, guard_bits)
            if fmt["fits"]:
                row += (
                    f" {('Q' + str(fmt['integer_bits']) + '.' + str(fmt['fractional_bits'])):>10}"
                    f" ({fmt['resolution']:.2e})"
                )
            else:
                row += f" {'overflow':>18}"
        print(row)


def print_verilog_guidance():
    print()
    print("Suggested fixed-point split for a first Verilog implementation")
    print("1. Keep state, measurements, innovation, and sigma points on a shared 16-bit format: Q8.8.")
    print("2. Keep covariance-like matrices P, Q, R, S, and Pxz on a shared 16-bit format: Q7.9.")
    print("3. Keep Kalman gain on a tighter signed format: Q3.13.")
    print("4. Use 32-bit products for all multiplies, then round and saturate back to the storage format.")
    print("5. Accumulate weighted sums and dot products with at least 4 extra integer bits beyond the storage format.")
    print("6. For a safer first RTL pass, use 24-bit internal accumulators and matrix storage: state Q8.16, covariance Q7.17, gain Q3.21.")
    print("7. Validate the chosen widths again after adding your real sensor model, wider motion envelope, and any matrix inversion approximation.")


def main():
    parser = argparse.ArgumentParser(
        description="Estimate fixed-point Q formats for UKF state and intermediate variables."
    )
    parser.add_argument("--trials", type=int, default=32, help="Number of random trials to sweep.")
    parser.add_argument("--time-steps", type=int, default=300, help="Simulation steps per trial.")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation timestep.")
    parser.add_argument(
        "--accel-std",
        type=float,
        default=0.35,
        help="Process acceleration noise standard deviation.",
    )
    parser.add_argument(
        "--measurement-std",
        type=float,
        nargs=3,
        default=(2.0, 2.0, 1.5),
        metavar=("SX", "SY", "SZ"),
        help="Measurement standard deviations for x, y, z.",
    )
    parser.add_argument(
        "--bits",
        type=int,
        nargs="+",
        default=(16, 24, 32),
        help="Total signed datapath widths to evaluate.",
    )
    parser.add_argument(
        "--guard-bits",
        type=int,
        default=1,
        help="Extra integer bits reserved above the observed range.",
    )
    args = parser.parse_args()

    if args.trials <= 0:
        raise ValueError("trials must be positive")
    if args.time_steps < 2:
        raise ValueError("time_steps must be at least 2")
    if args.dt <= 0.0:
        raise ValueError("dt must be positive")
    if any(bits <= 0 for bits in args.bits):
        raise ValueError("all datapath widths must be positive")
    if args.guard_bits < 0:
        raise ValueError("guard_bits must be non-negative")

    stats = analyze_ranges(
        trials=args.trials,
        time_steps=args.time_steps,
        dt=args.dt,
        accel_std=args.accel_std,
        measurement_std=np.asarray(args.measurement_std, dtype=float),
    )
    print_report(stats, total_bits=args.bits, guard_bits=args.guard_bits)
    print_verilog_guidance()


if __name__ == "__main__":
    main()
