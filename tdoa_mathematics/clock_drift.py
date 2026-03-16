"""
Clock drift analysis: t_tdoa vs t_pose.
Usage: python clock_drift.py [CSV_PATH] [--no-dedup]
Default: const1-trial1-tdoa2.csv
"""
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

DATASET_ROOT = Path(__file__).resolve().parent.parent / "dataset" / "flight-dataset"
DEFAULT_CSV  = DATASET_ROOT / "csv-data" / "const1" / "const1-trial1-tdoa2.csv"

def analyse(csv_path: Path, dedup: bool = True):
    df = pd.read_csv(csv_path)
    df = df.dropna(subset=["t_tdoa", "t_pose"])
    if dedup:
        df = df.drop_duplicates(subset="t_pose", keep="first")

    t_tdoa = df["t_tdoa"].values
    delta  = t_tdoa - df["t_pose"].values

    slope, intercept = np.polyfit(t_tdoa, delta, 1)
    drift_ppm   = slope * 1e6
    total_drift = slope * (t_tdoa[-1] - t_tdoa[0])

    print(f"File           : {csv_path.name}")
    print(f"Samples used   : {len(df)}")
    print(f"Duration       : {t_tdoa[-1] - t_tdoa[0]:.1f} s")
    print(f"Initial offset : {intercept*1000:+.3f} ms")
    print(f"Drift rate     : {drift_ppm:+.3f} ppm  ({slope:+.3e} s/s)")
    print(f"Total drift    : {total_drift*1000:+.3f} ms over recording")

    fitted = np.polyval([slope, intercept], t_tdoa)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.scatter(t_tdoa, delta * 1000, s=1, alpha=0.3, color="steelblue",
               label="t_tdoa − t_pose")
    ax.plot(t_tdoa, fitted * 1000, color="tomato", linewidth=1.5,
            label=f"Linear fit  ({drift_ppm:+.2f} ppm)")
    ax.set_xlabel("t_tdoa (s)")
    ax.set_ylabel("Clock difference (ms)")
    ax.set_title(f"Clock drift: UWB vs mocap — {csv_path.name}")
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout(); plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default=str(DEFAULT_CSV))
    parser.add_argument("--no-dedup", action="store_true")
    args = parser.parse_args()
    analyse(Path(args.csv), dedup=not args.no_dedup)

if __name__ == "__main__":
    main()
