#!/usr/bin/env python3
"""
DS-TWR 3D Trilateration Visualizer

Usage:
  python3 twr_trilateration_3D.py --live
  python3 twr_trilateration_3D.py --live --port /dev/ttyACM1
  python twr_trilateration_3D.py --sim (to see the graph visualization demo)
"""

import serial
import re
import math
import time
import random
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from itertools import combinations
from statistics import median
from collections import deque
from datetime import datetime


PORT = "/dev/ttyACM0"
BAUD = 115200

# CHANGE THIS when anchors / anchor positions change (units is in meters )
# Spread anchors in z (not all coplanar) or the z solution will be unstable.
ANCHORS = {
    1: (0.90, 0.60, 0.00),
    3: (0.90, 0.00, 1.20),
    6: (0.00, 0.60, 1.20),
    7: (0.00, 0.00, 0.00),
}

MIN_ANCHORS = 4
MAX_DISTANCE = 30.0
MIN_DISTANCE = 0.05
MAX_RESIDUAL = 1.0
MAX_SPREAD = 2.0
TRAIL_LENGTH = 200

PATTERN = re.compile(
    r"Anchor\s+(\d+):\s+(-?\d+\.?\d*)\s+m"
)


def trilaterate_4(a1, a2, a3, a4, d1, d2, d3, d4):
    x1, y1, z1 = a1
    x2, y2, z2 = a2
    x3, y3, z3 = a3
    x4, y4, z4 = a4

    A = np.array([
        [2 * (x2 - x1), 2 * (y2 - y1), 2 * (z2 - z1)],
        [2 * (x3 - x1), 2 * (y3 - y1), 2 * (z3 - z1)],
        [2 * (x4 - x1), 2 * (y4 - y1), 2 * (z4 - z1)],
    ])

    b = np.array([
        d1**2 - d2**2 + x2**2 + y2**2 + z2**2 - x1**2 - y1**2 - z1**2,
        d1**2 - d3**2 + x3**2 + y3**2 + z3**2 - x1**2 - y1**2 - z1**2,
        d1**2 - d4**2 + x4**2 + y4**2 + z4**2 - x1**2 - y1**2 - z1**2,
    ])

    try:
        sol = np.linalg.solve(A, b)
        return float(sol[0]), float(sol[1]), float(sol[2])
    except np.linalg.LinAlgError:
        return None


def residual_ok(x, y, z, anchor_ids, distances):
    for aid in anchor_ids:
        ax, ay, az = ANCHORS[aid]
        d_est = math.sqrt((x - ax)**2 + (y - ay)**2 + (z - az)**2)
        if abs(d_est - distances[aid]) > MAX_RESIDUAL:
            return False
    return True


def compute_position(distances):
    available = [aid for aid in distances if aid in ANCHORS]

    if len(available) < MIN_ANCHORS:
        return None

    solutions = []

    for combo in combinations(available, 4):
        a1, a2, a3, a4 = combo
        sol = trilaterate_4(
            ANCHORS[a1], ANCHORS[a2], ANCHORS[a3], ANCHORS[a4],
            distances[a1], distances[a2], distances[a3], distances[a4],
        )
        if sol is None:
            continue

        x, y, z = sol
        if residual_ok(x, y, z, combo, distances):
            solutions.append((x, y, z))

    if not solutions:
        return None

    xs = [s[0] for s in solutions]
    ys = [s[1] for s in solutions]
    zs = [s[2] for s in solutions]

    if (max(xs) - min(xs)) > MAX_SPREAD or \
       (max(ys) - min(ys)) > MAX_SPREAD or \
       (max(zs) - min(zs)) > MAX_SPREAD:
        return None

    return median(xs), median(ys), median(zs), len(solutions)


class LivePlot:
    def __init__(self):
        self.x_hist = deque(maxlen=TRAIL_LENGTH)
        self.y_hist = deque(maxlen=TRAIL_LENGTH)
        self.z_hist = deque(maxlen=TRAIL_LENGTH)
        self.count = 0

        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._setup_axes()

        self.trail_line, = self.ax.plot(
            [], [], [], "b-", alpha=0.4, linewidth=1.5, label="Trail"
        )
        self.pos_dot = self.ax.scatter(
            [], [], [], c="blue", s=120,
            marker="o", edgecolors="darkblue", linewidths=2,
            label="Tag",
        )

        ax_coords = list(ANCHORS.values())
        self.ax.scatter(
            [p[0] for p in ax_coords],
            [p[1] for p in ax_coords],
            [p[2] for p in ax_coords],
            c="red", s=200, marker="^",
            edgecolors="darkred", linewidths=2, label="Anchors",
        )
        for aid, (x, y, z) in ANCHORS.items():
            self.ax.text(x, y, z, f"  A{aid}", fontsize=10, fontweight="bold")

        self.info_text = self.ax.text2D(
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
        zs = [p[2] for p in ANCHORS.values()]
        margin = 1.0
        self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
        self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        self.ax.set_zlim(min(zs) - margin, max(zs) + margin)
        self.ax.set_xlabel("X (m)", fontsize=11, fontweight="bold")
        self.ax.set_ylabel("Y (m)", fontsize=11, fontweight="bold")
        self.ax.set_zlabel("Z (m)", fontsize=11, fontweight="bold")
        self.ax.set_title("DS-TWR Live 3D Position", fontsize=14, fontweight="bold")

    def update(self, x, y, z, n_anchors, n_quads):
        self.x_hist.append(x)
        self.y_hist.append(y)
        self.z_hist.append(z)
        self.count += 1

        self.trail_line.set_data(list(self.x_hist), list(self.y_hist))
        self.trail_line.set_3d_properties(list(self.z_hist))
        self.pos_dot._offsets3d = ([x], [y], [z])

        self.info_text.set_text(
            f"Position: ({x:.2f}, {y:.2f}, {z:.2f}) m\n"
            f"Sample: {self.count}\n"
            f"Anchors: {n_anchors}  Quads: {n_quads}"
        )

        return self.trail_line, self.pos_dot, self.info_text


def run_live(port, baud):
    print("=" * 60)
    print("DS-TWR 3D Trilateration - Live Mode")
    print("=" * 60)
    print(f"\nAnchors ({len(ANCHORS)}):")
    for aid, (x, y, z) in sorted(ANCHORS.items()):
        print(f"  A{aid}: ({x:.2f}, {y:.2f}, {z:.2f}) m")
    print(f"\nSerial: {port} @ {baud}")
    print("Close the plot window to stop.\n")

    ser = serial.Serial(port, baud, timeout=0.1)
    ser.reset_input_buffer()

    viz = LivePlot()
    distances = {}

    def update_frame(frame):
        nonlocal distances

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
                x, y, z, n_q = result
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(
                    f"{viz.count+1:05d} | {ts} | "
                    f"X={x:6.2f}  Y={y:6.2f}  Z={z:6.2f} m | "
                    f"Anchors={len(distances)}  Quads={n_q}"
                )
                return viz.update(x, y, z, len(distances), n_q)

        return viz.trail_line, viz.pos_dot, viz.info_text

    try:
        ani = FuncAnimation(
            viz.fig, update_frame,
            interval=50, blit=False, cache_frame_data=False,
        )
        plt.show()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
        plt.close("all")
        print("Serial closed.")


def run_sim():
    print("DS-TWR 3D Trilateration - Sim Mode")
    print(f"Anchors ({len(ANCHORS)}):")
    for aid, (x, y, z) in sorted(ANCHORS.items()):
        print(f"  A{aid}: ({x:.2f}, {y:.2f}, {z:.2f}) m")
    print("Close the plot window to stop.\n")

    viz = LivePlot()

    xs = [p[0] for p in ANCHORS.values()]
    ys = [p[1] for p in ANCHORS.values()]
    zs = [p[2] for p in ANCHORS.values()]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    cz = (min(zs) + max(zs)) / 2
    rx = max(0.3, (max(xs) - min(xs)) / 3)
    ry = max(0.3, (max(ys) - min(ys)) / 3)
    rz = max(0.3, (max(zs) - min(zs)) / 3)

    NOISE = 0.05
    t0 = time.time()

    def update_frame(frame):
        t = time.time() - t0
        gx = cx + rx * math.cos(0.6 * t)
        gy = cy + ry * math.sin(0.6 * t)
        gz = cz + rz * math.sin(0.3 * t)

        distances = {}
        for aid, (ax, ay, az) in ANCHORS.items():
            d = math.sqrt((gx - ax)**2 + (gy - ay)**2 + (gz - az)**2)
            distances[aid] = d + random.gauss(0, NOISE)

        result = compute_position(distances)
        if result is not None:
            x, y, z, n_q = result
            print(
                f"{viz.count+1:05d} | "
                f"truth=({gx:5.2f},{gy:5.2f},{gz:5.2f}) "
                f"est=({x:5.2f},{y:5.2f},{z:5.2f}) m | Quads={n_q}"
            )
            return viz.update(x, y, z, len(distances), n_q)

        return viz.trail_line, viz.pos_dot, viz.info_text

    try:
        ani = FuncAnimation(
            viz.fig, update_frame,
            interval=80, blit=False, cache_frame_data=False,
        )
        plt.show()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        plt.close("all")


def main():
    parser = argparse.ArgumentParser(
        description="DS-TWR 3D Trilateration Visualizer",
    )
    parser.add_argument("--live", action="store_true", help="Live from serial")
    parser.add_argument("--sim", action="store_true", help="Synthetic motion (no hardware)")
    parser.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Baud rate (default: {BAUD})")

    args = parser.parse_args()

    if args.sim:
        run_sim()
    elif args.live:
        run_live(args.port, args.baud)
    else:
        parser.print_help()
        print("\nRun with --sim to preview the plot, or --live for hardware.")


if __name__ == "__main__":
    main()
