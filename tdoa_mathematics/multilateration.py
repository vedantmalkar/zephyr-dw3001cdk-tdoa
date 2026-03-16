"""
TDOA Multilateration Solver
============================
Reads anchor positions from survey-results/ and TDOA measurements from csv-data/,
solves for tag position using nonlinear least-squares, and writes a comparison CSV
with estimated vs. ground-truth coordinates side by side.

Usage:
    python multilateration.py [--const CONST] [--trial TRIAL] [--tdoa-type {2,3}]
                              [--window-ms MS] [--dataset-root PATH]

Defaults: all constellations, all trials, tdoa2, 100 ms window.

Output: tdoa_mathematics/results/<const>_<trial>_tdoa<type>_results.csv
"""

import argparse
import os
import sys
import re
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SPEED_OF_LIGHT = 299_702_547.0   # m/s in air (vacuum ~299_792_458)
MAX_TDOA_VALID  = 12.0            # metres — discard obvious outliers (room diagonal ~14 m)

DATASET_ROOT = Path(__file__).resolve().parent.parent / "dataset" / "flight-dataset"
SURVEY_DIR   = DATASET_ROOT / "survey-results"
CSV_DIR      = DATASET_ROOT / "csv-data"
RESULTS_DIR  = Path(__file__).resolve().parent / "results"


# ---------------------------------------------------------------------------
# Load anchor positions from survey txt
# ---------------------------------------------------------------------------
def load_anchors(constellation: int) -> dict[int, np.ndarray]:
    """Return dict {anchor_id: np.array([x, y, z])} for a constellation."""
    path = SURVEY_DIR / f"anchor_const{constellation}_survey.txt"
    if not path.exists():
        raise FileNotFoundError(f"Survey file not found: {path}")

    anchors = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("an") and "_quat" in line:
                continue
            m = re.match(r"an(\d+)_p,(.+)", line)
            if m:
                aid = int(m.group(1))
                coords = [float(v) for v in m.group(2).split(",")]
                anchors[aid] = np.array(coords)
    return anchors


# ---------------------------------------------------------------------------
# TDOA residual function for the solver
# ---------------------------------------------------------------------------
def tdoa_residuals(pos: np.ndarray,
                   anchor_A_positions: np.ndarray,
                   anchor_B_positions: np.ndarray,
                   measurements: np.ndarray) -> np.ndarray:
    """
    For each measurement i:
        tdoa_meas_i = d(tag, B_i) - d(tag, A_i)
        r_i = (||pos - B_i|| - ||pos - A_i||) - tdoa_meas_i

    Sign verified against the Crazyflie TDOA2 dataset convention.
    All values in metres.
    """
    dist_A = np.linalg.norm(anchor_A_positions - pos, axis=1)   # (N,)
    dist_B = np.linalg.norm(anchor_B_positions - pos, axis=1)
    return (dist_B - dist_A) - measurements


# ---------------------------------------------------------------------------
# Solve for one time window
# ---------------------------------------------------------------------------
def solve_position(anchor_A_pos: np.ndarray,
                   anchor_B_pos: np.ndarray,
                   measurements: np.ndarray,
                   initial_guess: np.ndarray,
                   anchor_positions: np.ndarray) -> tuple[np.ndarray | None, float]:
    """
    Run nonlinear least-squares TDOA solver with multi-start to escape local minima.
    Tries: warm-start from previous solution, anchor centroid, and a grid of
    candidate points spanning the anchor bounding box.
    Returns (estimated_position, rms_residual) or (None, inf) on failure.
    """
    if len(measurements) < 3:
        return None, np.inf

    # Build candidate starting points
    x_min, y_min, z_min = anchor_positions.min(axis=0)
    x_max, y_max, z_max = anchor_positions.max(axis=0)
    centroid = anchor_positions.mean(axis=0)

    # 3x3x3 grid spanning the anchor bounding box + warm start
    xs = np.linspace(x_min, x_max, 3)
    ys = np.linspace(y_min, y_max, 3)
    zs = np.linspace(z_min, z_max, 3)
    grid_starts = [np.array([x, y, z]) for x in xs for y in ys for z in zs]
    candidates = [initial_guess, centroid] + grid_starts

    best_pos, best_cost = None, np.inf
    for x0 in candidates:
        try:
            res = least_squares(
                tdoa_residuals,
                x0=x0,
                args=(anchor_A_pos, anchor_B_pos, measurements),
                method="lm",
                max_nfev=500,
            )
            if res.cost < best_cost:
                best_cost = res.cost
                best_pos  = res.x
        except Exception:
            continue

    if best_pos is not None:
        rms = np.sqrt(best_cost / max(len(measurements), 1))
        return best_pos, rms
    return None, np.inf


# ---------------------------------------------------------------------------
# Process one CSV file
# ---------------------------------------------------------------------------
def process_file(csv_path: Path,
                 anchors: dict[int, np.ndarray],
                 window_ms: float = 100.0) -> pd.DataFrame:
    """
    Group measurements into time windows, solve for position in each window,
    and return a DataFrame with estimated and ground-truth coordinates.
    """
    df = pd.read_csv(csv_path)

    # Drop rows with missing TDOA or anchor IDs
    df = df.dropna(subset=["tdoa_meas", "idA", "idB"])
    df["idA"] = df["idA"].astype(int)
    df["idB"] = df["idB"].astype(int)

    # Filter out physically impossible measurements
    df = df[df["tdoa_meas"].abs() <= MAX_TDOA_VALID]

    # Drop rows referencing anchors not in the survey
    known = set(anchors.keys())
    df = df[df["idA"].isin(known) & df["idB"].isin(known)]

    if df.empty:
        print(f"  [WARN] No valid rows after filtering in {csv_path.name}")
        return pd.DataFrame()

    # Anchor positions array (for multi-start bounding box)
    anchor_array    = np.array(list(anchors.values()))
    anchor_centroid = anchor_array.mean(axis=0)

    # Ground truth: forward-fill pose (pose is broadcast alongside TDOA rows)
    gt_cols = ["t_pose", "pose_x", "pose_y", "pose_z"]
    df_gt = df.dropna(subset=["pose_x", "pose_y", "pose_z"])[["t_tdoa"] + gt_cols].copy()

    window_s = window_ms / 1000.0
    t_min = df["t_tdoa"].min()
    t_max = df["t_tdoa"].max()

    records = []
    t = t_min
    while t < t_max:
        t_end = t + window_s
        mask = (df["t_tdoa"] >= t) & (df["t_tdoa"] < t_end)
        window_df = df[mask]

        if len(window_df) >= 3:
            # Build anchor position arrays for solver
            A_pos = np.array([anchors[a] for a in window_df["idA"]])
            B_pos = np.array([anchors[b] for b in window_df["idB"]])
            meas  = window_df["tdoa_meas"].values

            # Warm-start: use previous estimate only if it was close (< 1m change)
            if records:
                last  = records[-1]
                guess = np.array([last["est_x"], last["est_y"], last["est_z"]])
                if not np.all(np.isfinite(guess)):
                    guess = anchor_centroid
            else:
                guess = anchor_centroid

            est_pos, rms = solve_position(A_pos, B_pos, meas, guess, anchor_array)

            # Ground truth: nearest pose to window centre
            t_mid = (t + t_end) / 2.0
            gt_mask = (df_gt["t_tdoa"] >= t) & (df_gt["t_tdoa"] < t_end)
            gt_window = df_gt[gt_mask]
            if gt_window.empty:
                # Find nearest row outside window
                idx = (df_gt["t_tdoa"] - t_mid).abs().idxmin()
                gt_row = df_gt.loc[idx]
            else:
                # Median of poses in window (smooths sub-window motion)
                gt_row = gt_window[["pose_x", "pose_y", "pose_z"]].median()

            gt_x = float(gt_row["pose_x"]) if "pose_x" in gt_row else float(gt_row.iloc[0])
            gt_y = float(gt_row["pose_y"]) if "pose_y" in gt_row else float(gt_row.iloc[1])
            gt_z = float(gt_row["pose_z"]) if "pose_z" in gt_row else float(gt_row.iloc[2])

            if est_pos is not None:
                err_3d = float(np.linalg.norm(est_pos - np.array([gt_x, gt_y, gt_z])))
                records.append({
                    "time_s":       round(t_mid, 4),
                    "est_x":        round(float(est_pos[0]), 4),
                    "est_y":        round(float(est_pos[1]), 4),
                    "est_z":        round(float(est_pos[2]), 4),
                    "gt_x":         round(gt_x, 4),
                    "gt_y":         round(gt_y, 4),
                    "gt_z":         round(gt_z, 4),
                    "error_3d_m":   round(err_3d, 4),
                    "rms_residual": round(rms, 4),
                    "n_meas":       len(window_df),
                })
        t = t_end

    if not records:
        return pd.DataFrame()

    return pd.DataFrame(records)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="TDOA Multilateration Solver")
    parser.add_argument("--const",     type=int, default=None,
                        help="Constellation number (1-4). Default: all.")
    parser.add_argument("--trial",     type=int, default=None,
                        help="Trial number. Default: all.")
    parser.add_argument("--tdoa-type", type=int, choices=[2, 3], default=2,
                        help="TDOA type: 2 (ring, cleaner) or 3 (full mesh). Default: 2.")
    parser.add_argument("--window-ms", type=float, default=100.0,
                        help="Time window for grouping measurements (ms). Default: 100.")
    parser.add_argument("--dataset-root", type=str, default=None,
                        help="Override path to flight-dataset root.")
    args = parser.parse_args()

    global DATASET_ROOT, SURVEY_DIR, CSV_DIR
    if args.dataset_root:
        DATASET_ROOT = Path(args.dataset_root)
        SURVEY_DIR   = DATASET_ROOT / "survey-results"
        CSV_DIR      = DATASET_ROOT / "csv-data"

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    # Discover constellations
    constellations = sorted([
        int(d.name.replace("const", ""))
        for d in CSV_DIR.iterdir()
        if d.is_dir() and d.name.startswith("const")
    ])
    if args.const is not None:
        constellations = [args.const]

    total_files = 0
    all_summaries = []

    for const_id in constellations:
        print(f"\n=== Constellation {const_id} ===")
        try:
            anchors = load_anchors(const_id)
        except FileNotFoundError as e:
            print(f"  [SKIP] {e}")
            continue
        print(f"  Anchors loaded: {sorted(anchors.keys())}")

        const_dir = CSV_DIR / f"const{const_id}"
        pattern   = f"*tdoa{args.tdoa_type}.csv"
        csv_files = sorted(const_dir.glob(pattern))

        if args.trial is not None:
            csv_files = [f for f in csv_files if f"trial{args.trial}" in f.name]

        if not csv_files:
            print(f"  [WARN] No files matching {pattern} in {const_dir}")
            continue

        for csv_path in csv_files:
            print(f"  Processing {csv_path.name} ...", end=" ", flush=True)

            results_df = process_file(csv_path, anchors, window_ms=args.window_ms)

            if results_df.empty:
                print("no results.")
                continue

            # Compute summary stats
            n         = len(results_df)
            mae       = results_df["error_3d_m"].mean()
            rmse      = np.sqrt((results_df["error_3d_m"] ** 2).mean())
            p50       = results_df["error_3d_m"].median()
            p90       = results_df["error_3d_m"].quantile(0.90)

            print(f"{n} windows | MAE={mae:.3f}m  RMSE={rmse:.3f}m  "
                  f"P50={p50:.3f}m  P90={p90:.3f}m")

            # Save per-file result CSV
            stem        = csv_path.stem
            out_name    = f"{stem}_results.csv"
            out_path    = RESULTS_DIR / out_name
            results_df.to_csv(out_path, index=False)

            all_summaries.append({
                "file":         csv_path.name,
                "constellation": const_id,
                "windows":      n,
                "mae_m":        round(mae, 4),
                "rmse_m":       round(rmse, 4),
                "p50_m":        round(p50, 4),
                "p90_m":        round(p90, 4),
            })
            total_files += 1

    # Write overall summary
    if all_summaries:
        summary_path = RESULTS_DIR / f"summary_tdoa{args.tdoa_type}.csv"
        pd.DataFrame(all_summaries).to_csv(summary_path, index=False)
        print(f"\nSummary written to {summary_path}")
        print(f"Processed {total_files} file(s). Results in {RESULTS_DIR}/")
    else:
        print("\nNo results produced.")


if __name__ == "__main__":
    main()
