#!/usr/bin/env python3
"""
plot estimated trajectories aligned to ground truth.

Usage:
  python3 plot_trajectory.py <gt.txt> \
    -t <traj1.txt> "<label1>" \
    -t <traj2.txt> "<label2>" \
    [-t ...] \
    -o <out.pdf>

Example (3-panel figure for the paper):
  python3 plot_trajectory.py ~/datasets/gt_official.txt \
    -t ~/datasets/rtbench_experiments/paper_rerun/solo_t1.txt         "Solo" \
    -t ~/datasets/rtbench_experiments/paper_rerun/uncontrolled_t2.txt "Uncontrolled" \
    -t ~/datasets/rtbench_experiments/paper_rerun/controlled_t1.txt   "Controlled" \
    -o trajectory_comparison.pdf
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_tum(path):
    data = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                data.append([float(x) for x in parts[:8]])
    arr = np.array(data)
    if arr.size == 0:
        raise ValueError(f"no TUM rows found in {path}")
    return arr[:, 0], arr[:, 1:4]


def associate(ts_gt, ts_est, max_diff=0.02):
    matches = []
    for i, t_est in enumerate(ts_est):
        diffs = np.abs(ts_gt - t_est)
        j = int(np.argmin(diffs))
        if diffs[j] < max_diff:
            matches.append((i, j))
    return matches


def umeyama(src, dst):
    assert src.shape == dst.shape
    n, dim = src.shape
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    cov = dc.T @ sc / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(dim)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[dim - 1, dim - 1] = -1
    R = U @ S @ Vt
    var_s = np.sum(sc ** 2) / n
    scale = np.trace(np.diag(D) @ S) / var_s
    t = mu_d - scale * R @ mu_s
    return scale, R, t


def align_to_gt(gt_path, est_path):
    """Returns (full aligned est trajectory, full gt trajectory, rmse,
    n matched, n total est) or (None, None, None, n, total) if too few matches."""
    ts_gt, pos_gt = load_tum(gt_path)
    ts_est, pos_est = load_tum(est_path)
    matches = associate(ts_gt, ts_est)
    if len(matches) < 10:
        return None, pos_gt, None, len(matches), len(ts_est)
    i_est = np.array([m[0] for m in matches])
    i_gt = np.array([m[1] for m in matches])
    s, R, t = umeyama(pos_est[i_est], pos_gt[i_gt])
    # apply transform to the FULL estimated trajectory so the plotted line
    # is complete, not just the matched samples
    pos_est_aligned = s * (R @ pos_est.T).T + t
    # RMSE is computed over matched samples (same convention as evaluate_all.py)
    errs = np.linalg.norm(pos_est_aligned[i_est] - pos_gt[i_gt], axis=1)
    rmse = float(np.sqrt(np.mean(errs ** 2)))
    return pos_est_aligned, pos_gt, rmse, len(matches), len(ts_est)


AXES_OPTIONS = {
    "xy": (0, 1, "X (m)", "Y (m)"),
    "xz": (0, 2, "X (m)", "Z (m)"),
    "yz": (1, 2, "Y (m)", "Z (m)"),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("gt", help="ground-truth TUM file")
    ap.add_argument("-t", "--trajectory", nargs=2, action="append", metavar=("PATH", "LABEL"), 
                    help="estimated trajectory and its label (repeatable)")
    ap.add_argument("-o", "--out", required=True, help="output PDF or PNG")
    ap.add_argument("--axes", default="xy", choices=list(AXES_OPTIONS),
                    help="2D projection (default: xy)")
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    if not args.trajectory:
        sys.exit("at least one -t PATH LABEL is required")

    ix, iy, lx, ly = AXES_OPTIONS[args.axes]

    results = []
    for path, label in args.trajectory:
        if not os.path.isfile(path):
            print(f"WARN: {path} not found, skipping", file=sys.stderr)
            continue
        try:
            est, gt, rmse, matched, total = align_to_gt(args.gt, path)
        except Exception as e:
            print(f"WARN: {path}: {e}", file=sys.stderr)
            continue
        if est is None:
            print(f"WARN: {path}: only {matched} timestamp matches, skipping",
                  file=sys.stderr)
            continue
        results.append((label, est, gt, rmse, total))

    if not results:
        sys.exit("no trajectories could be aligned")

    n = len(results)
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 5), squeeze=False)
    axes = axes[0]

    est_colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd"]

    # uniform axis limits across panels for fair visual comparison
    all_x = np.concatenate([results[0][2][:, ix]] + [r[1][:, ix] for r in results])
    all_y = np.concatenate([results[0][2][:, iy]] + [r[1][:, iy] for r in results])
    pad = 0.1 * max(all_x.max() - all_x.min(), all_y.max() - all_y.min())
    xlim = (all_x.min() - pad, all_x.max() + pad)
    ylim = (all_y.min() - pad, all_y.max() + pad)

    for k, (label, est, gt, rmse, total) in enumerate(results):
        ax = axes[k]
        ax.plot(gt[:, ix], gt[:, iy], color="black", lw=1.5, label="Ground truth", zorder=1)
        ax.plot(est[:, ix], est[:, iy], color=est_colors[k % len(est_colors)], lw=1.2, label=f"{label}", zorder=2)
        ax.scatter([gt[0, ix]], [gt[0, iy]], color="black", s=50, zorder=5, label="Start")
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel(lx)
        ax.set_ylabel(ly)
        ax.set_title(f"{label}\nRMSE: {rmse:.3f} m  ({total} poses)", fontsize=11)
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=9)

    if args.title:
        fig.suptitle(args.title, fontsize=13)
        fig.tight_layout(rect=[0, 0, 1, 0.96])
    else:
        fig.tight_layout()
    fig.savefig(args.out, dpi=200, bbox_inches="tight")
    print(f"wrote {args.out}")
    for label, _, _, rmse, total in results:
        print(f"  {label}: {rmse:.4f} m  ({total} poses)")


if __name__ == "__main__":
    main()