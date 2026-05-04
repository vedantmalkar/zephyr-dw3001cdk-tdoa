#!/usr/bin/env python3
"""
DS-TWR Trilateration Visualizer

Reads distance measurements from the tag over serial,
trilaterates the tag position from anchor distances,
and plots it live.

Usage:
  python3 twr_trilateration.py --live
  python3 twr_trilateration.py --live --port /dev/ttyACM1
"""

import serial
import re
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from itertools import combinations
from statistics import median
from collections import deque
from datetime import datetime


PORT = "/dev/ttyACM0"
BAUD = 115200

# Anchor positions in meters (x, y)
# Change these to match your physical setup
ANCHORS = {
    1: (0.90, 0.60),
    3: (0.90, 0.00),
    6: (0.00, 0.60),
    7: (0.00, 0.00),
}

MIN_ANCHORS = 3          # need at least 3 for trilateration
MAX_DISTANCE = 30.0      # reject distances above this (meters)
MIN_DISTANCE = 0.05      # reject distances below this (meters)
MAX_RESIDUAL = 1.0        # max error per anchor to accept a solution (meters)
MAX_SPREAD = 2.0          # max spread between triplet solutions (meters)
TRAIL_LENGTH = 200        # how many past positions to show

# Regex to parse: [00:00:55.474,365] <inf> ds_twr: Anchor 6: 0.30 m  seq=0
PATTERN = re.compile(
    r"Anchor\s+(\d+):\s+(-?\d+\.?\d*)\s+m"
)

# ===================== TRILATERATION =====================

def trilaterate_3(a1, a2, a3, d1, d2, d3):
    """Solve 2D position from 3 anchor positions and distances."""
    x1, y1 = a1
    x2, y2 = a2
    x3, y3 = a3

    A = np.array([
        [2 * (x2 - x1), 2 * (y2 - y1)],
        [2 * (x3 - x1), 2 * (y3 - y1)],
    ])

    b = np.array([
        d1**2 - d2**2 + x2**2 + y2**2 - x1**2 - y1**2,
        d1**2 - d3**2 + x3**2 + y3**2 - x1**2 - y1**2,
    ])

    try:
        sol = np.linalg.solve(A, b)
        return float(sol[0]), float(sol[1])
    except np.linalg.LinAlgError:
        return None


def residual_ok(x, y, anchor_ids, distances):
    """Check that the solution is consistent with all used anchors."""
    for aid in anchor_ids:
        ax, ay = ANCHORS[aid]
        d_est = math.hypot(x - ax, y - ay)
        if abs(d_est - distances[aid]) > MAX_RESIDUAL:
            return False
    return True


def compute_position(distances):
    """
    Try all combinations of 3 anchors, trilaterate each,
    filter by residual, return median of good solutions.
    """
    available = [aid for aid in distances if aid in ANCHORS]

    if len(available) < MIN_ANCHORS:
        return None

    solutions = []

    for combo in combinations(available, 3):
        a1, a2, a3 = combo
        sol = trilaterate_3(
            ANCHORS[a1], ANCHORS[a2], ANCHORS[a3],
            distances[a1], distances[a2], distances[a3],
        )
        if sol is None:
            continue

        x, y = sol
        if residual_ok(x, y, combo, distances):
            solutions.append((x, y))

    if not solutions:
        return None

    xs = [s[0] for s in solutions]
    ys = [s[1] for s in solutions]

    if (max(xs) - min(xs)) > MAX_SPREAD or (max(ys) - min(ys)) > MAX_SPREAD:
        return None

    return median(xs), median(ys), len(solutions)


# ===================== VISUALIZATION =====================

class LivePlot:
    def __init__(self):
        self.x_hist = deque(maxlen=TRAIL_LENGTH)
        self.y_hist = deque(maxlen=TRAIL_LENGTH)
        self.count = 0

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self._setup_axes()

        # trail + current position
        self.trail_line, = self.ax.plot(
            [], [], "b-", alpha=0.3, linewidth=1.5, label="Trail"
        )
        self.pos_dot = self.ax.scatter(
            [], [], c="blue", s=120, zorder=5,
            marker="o", edgecolors="darkblue", linewidths=2,
            label="Tag",
        )

        # draw anchors
        ax_coords = list(ANCHORS.values())
        self.ax.scatter(
            [p[0] for p in ax_coords],
            [p[1] for p in ax_coords],
            c="red", s=250, marker="^", zorder=10,
            edgecolors="darkred", linewidths=2, label="Anchors",
        )
        for aid, (x, y) in ANCHORS.items():
            self.ax.annotate(
                f"A{aid}",
                (x, y),
                xytext=(8, 8),
                textcoords="offset points",
                fontsize=10,
                fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.9),
            )

        self.info_text = self.ax.text(
            0.02, 0.98, "",
            transform=self.ax.transAxes,
            verticalalignment="top",
            fontsize=10,
            bbox=dict(boxstyle="round", facecolor="lightgreen", alpha=0.9),
        )

        self.ax.legend(loc="upper right", fontsize=10)

    def _setup_axes(self):
        xs = [p[0] for p in ANCHORS.values()]
        ys = [p[1] for p in ANCHORS.values()]
        margin = 1.0
        self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        self.ax.set_xlabel("X (m)", fontsize=12, fontweight="bold")
        self.ax.set_ylabel("Y (m)", fontsize=12, fontweight="bold")
        self.ax.set_title("DS-TWR Live Position", fontsize=14, fontweight="bold")
        self.ax.grid(True, alpha=0.3, linestyle="--")
        self.ax.set_aspect("equal")

    def update(self, x, y, n_anchors, n_triplets):
        self.x_hist.append(x)
        self.y_hist.append(y)
        self.count += 1

        self.trail_line.set_data(list(self.x_hist), list(self.y_hist))
        self.pos_dot.set_offsets([[x, y]])

        self.info_text.set_text(
            f"Position: ({x:.2f}, {y:.2f}) m\n"
            f"Sample: {self.count}\n"
            f"Anchors: {n_anchors}  Triplets: {n_triplets}"
        )

        return self.trail_line, self.pos_dot, self.info_text


# ===================== SERIAL READER =====================

def run_live(port, baud):
    print("=" * 60)
    print("DS-TWR Trilateration - Live Mode")
    print("=" * 60)
    print(f"\nAnchors ({len(ANCHORS)}):")
    for aid, (x, y) in sorted(ANCHORS.items()):
        print(f"  A{aid}: ({x:.2f}, {y:.2f}) m")
    print(f"\nSerial: {port} @ {baud}")
    print("Close the plot window to stop.\n")

    ser = serial.Serial(port, baud, timeout=0.1)
    ser.reset_input_buffer()

    viz = LivePlot()
    distances = {}

    def update_frame(frame):
        nonlocal distances

        # read several lines per frame for responsiveness
        for _ in range(20):
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
            except Exception:
                continue

            m = PATTERN.search(line)
            if not m:
                continue

            anchor_id = int(m.group(1))
            dist = float(m.group(2))

            if anchor_id not in ANCHORS:
                continue
            if not (MIN_DISTANCE <= dist <= MAX_DISTANCE):
                continue

            distances[anchor_id] = dist

            result = compute_position(distances)
            if result is not None:
                x, y, n_tri = result
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(
                    f"{viz.count+1:05d} | {ts} | "
                    f"X={x:6.2f} m  Y={y:6.2f} m | "
                    f"Anchors={len(distances)}  Triplets={n_tri}"
                )
                return viz.update(x, y, len(distances), n_tri)

        return viz.trail_line, viz.pos_dot, viz.info_text

    try:
        ani = FuncAnimation(
            viz.fig, update_frame,
            interval=50, blit=True, cache_frame_data=False,
        )
        plt.show()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
        print("Serial closed.")


# ===================== MAIN =====================

def main():
    parser = argparse.ArgumentParser(
        description="DS-TWR Trilateration Visualizer",
    )
    parser.add_argument("--live", action="store_true", help="Live from serial")
    parser.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Baud rate (default: {BAUD})")

    args = parser.parse_args()

    if args.live:
        run_live(args.port, args.baud)
    else:
        parser.print_help()
        print("\nRun with --live to start.")


if __name__ == "__main__":
    main()
