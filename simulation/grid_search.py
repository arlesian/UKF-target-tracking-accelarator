import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
import simul
import numpy as np

def run_once(alpha, beta, kappa, seed=4, steps=300):
    np.random.seed(seed)
    dt = 0.1
    ukf = simul.UKF(6, alpha=alpha, beta=beta, kappa=kappa)

    true_pos, est_pos = [], []
    prev_range = None

    for k in range(steps):
        t = k * dt
        ego = simul.ego_trajectory(t)
        tgt = simul.true_trajectory(t)

        rel = tgt - ego[:3]

        r = np.linalg.norm(rel) + 1e-6
        if prev_range is None:
            r_dot_meas = 0.0
        else:
            r_dot_meas = (r - prev_range) / dt
        prev_range = r

        v_along_rel = (r_dot_meas * rel / r) if r > 1e-9 else np.zeros(3)

        z_radar = simul.cartesian_to_spherical(*rel, *v_along_rel) + \
                  np.random.multivariate_normal(np.zeros(4), ukf.R_radar)

        z_ir = simul.cartesian_to_ir(*rel) + \
               np.random.multivariate_normal(np.zeros(2), ukf.R_ir)

        ukf.predict(dt)
        ukf.update(z_radar, z_ir, ego)

        true_pos.append(tgt)
        est_pos.append(ukf.x[:3])

    true_pos = np.array(true_pos)
    est_pos = np.array(est_pos)
    rmse = np.sqrt(np.mean(np.sum((true_pos - est_pos)**2, axis=1)))
    return rmse

if __name__ == '__main__':
    n = 6
    alphas = [1e-4, 1e-3, 1e-2, 0.05, 0.1, 0.2]
    betas = [0, 1, 2]
    kappas = [3-n, 0, 1, 3, 6]

    results = []
    for a in alphas:
        for b in betas:
            for k in kappas:
                try:
                    rmse = run_once(a, b, k)
                except Exception as e:
                    rmse = float('inf')
                results.append(((a, b, k), rmse))
                print(f"alpha={a}, beta={b}, kappa={k} -> RMSE={rmse:.3f}")

    results.sort(key=lambda x: x[1])
    print("\nTop 5 results:")
    for (p, r) in results[:5]:
        print(p, r)

    # save results
    out = os.path.join(os.path.dirname(__file__), 'grid_search_results.txt')
    with open(out, 'w') as f:
        for (p, r) in results:
            f.write(f"{p} {r}\n")
    print('\nSaved full results to', out)
