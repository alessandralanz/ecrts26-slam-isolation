#!/usr/bin/env python3
import sys, os, glob
import numpy as np
def load_tum(path):
    data = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 8:
                data.append([float(x) for x in parts[:8]])
    arr = np.array(data)
    return arr[:, 0], arr[:, 1:4]
def associate(ts_gt, ts_est, max_diff=0.02):
    matches = []
    for i, t_est in enumerate(ts_est):
        diffs = np.abs(ts_gt - t_est)
        j = np.argmin(diffs)
        if diffs[j] < max_diff:
            matches.append((i, j))
    return matches
def umeyama_alignment(src, dst):
    assert src.shape == dst.shape
    n, dim = src.shape
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    cov = dst_c.T @ src_c / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(dim)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[dim - 1, dim - 1] = -1
    R = U @ S @ Vt
    var_src = np.sum(src_c ** 2) / n
    scale = np.trace(np.diag(D) @ S) / var_src
    t = mu_dst - scale * R @ mu_src
    return scale, R, t
def compute_ape_rmse(gt_path, est_path):
    ts_gt, pos_gt = load_tum(gt_path)
    ts_est, pos_est = load_tum(est_path)
    matches = associate(ts_gt, ts_est)
    if len(matches) < 10:
        return None, len(matches), len(ts_est)
    idx_est = np.array([m[0] for m in matches])
    idx_gt = np.array([m[1] for m in matches])
    pos_est_m = pos_est[idx_est]
    pos_gt_m = pos_gt[idx_gt]
    scale, R, t = umeyama_alignment(pos_est_m, pos_gt_m)
    pos_est_aligned = scale * (R @ pos_est_m.T).T + t
    errors = np.linalg.norm(pos_est_aligned - pos_gt_m, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    return rmse, len(matches), len(ts_est)
if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <ground_truth.txt> <trajectory_dir_or_file> [pattern]")
        sys.exit(1)
    gt_path = sys.argv[1]
    target = sys.argv[2]
    pattern = sys.argv[3] if len(sys.argv) > 3 else '*.txt'
    if os.path.isfile(target):
        files = [target]
    else:
        files = sorted(glob.glob(os.path.join(target, pattern)))
    if not files:
        print(f"No files found matching {target}/{pattern}")
        sys.exit(1)
    print(f"{'File':<45} {'Poses':>6} {'Matched':>8} {'RMSE (m)':>10}")
    print("-" * 75)
    results = []
    for f in files:
        name = os.path.basename(f)
        rmse, matched, total = compute_ape_rmse(gt_path, f)
        if rmse is not None:
            print(f"{name:<45} {total:>6} {matched:>8} {rmse:>10.4f}")
            results.append((name, total, matched, rmse))
        else:
            print(f"{name:<45} {total:>6} {matched:>8} {'FAIL':>10}")
    if results:
        rmses = [r[3] for r in results]
        print("-" * 75)
        print(f"{'SUMMARY':<45} {'':>6} {'':>8} {'':>10}")
        print(f"Mean RMSE: {np.mean(rmses):.4f} m")
        print(f"Min RMSE: {np.min(rmses):.4f} m")
        print(f"Max RMSE: {np.max(rmses):.4f} m")
        print(f"Std RMSE: {np.std(rmses):.4f} m")
        print(f"N trials: {len(rmses)}")
