#!/usr/bin/env python3
"""
BLE TDoA Multi-Slave Client

Scans for all DWM3001-TDOA devices, connects to every one concurrently,
and streams data from all of them tagged by device address.

Each board runs the same ble_tdoa_slave firmware.  No firmware changes needed.

Output format:
  [HH:MM:SS.mmm] [AA:BB:CC:DD:EE:FF] SYNC  seq=N  tx=...  rx=...  offset=...  drift=...  corrected=...
  [HH:MM:SS.mmm] [AA:BB:CC:DD:EE:FF] BLINK rx=...  master_time=...

CSV log columns (--log):
  time, addr, type, seq, tx_ts, rx_ts, offset, drift, corrected, master_time

Usage:
  python ble_tdoa_multi_client.py
  python ble_tdoa_multi_client.py --scan-time 10 --log tdoa.csv
    python ble_tdoa_multi_client.py --anchor 10:0,0 --anchor 11:5,0 --anchor 12:0,4
"""

import asyncio
import argparse
import csv
import math
import sys
import time
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME   = "DWM3001-TDOA"
NUS_TX_UUID   = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
RECONNECT_SEC = 5          # seconds to wait before reconnect attempt

# DW3000 timestamp period in seconds.
DWT_TIME_UNIT_S = 1.0 / (499.2e6 * 128.0)
SPEED_OF_LIGHT_M_S = 299_792_458.0

# Aggregate same BLINK observed by multiple anchors:
# key = (sync_seq, blink_seq), value = {"t0": float, "anchors": {id: corrected}, "reported": int}
blink_groups = {}
BLINK_GROUP_TTL_SEC = 5.0

# Anchor geometry defaults: {anchor_id: (x_m, y_m)}.
anchor_positions = {
    7: (0.0, 0.0),
    0: (0.9, 0.0),
    6: (90.0, 60.0),
}


def parse_args():
    p = argparse.ArgumentParser(description="Connect to multiple DWM3001-TDOA slaves over BLE")
    p.add_argument("--scan-time", type=float, default=5.0,
                   metavar="SEC", help="Initial scan duration in seconds (default: 5)")
    p.add_argument("--log", metavar="FILE", help="CSV file to log all data (appended)")
    p.add_argument(
        "--anchor", action="append", default=[], metavar="ID:X,Y",
        help="Anchor position mapping, e.g. --anchor 10:0,0 (repeat for each anchor)"
    )
    return p.parse_args()


def parse_anchor_positions(items):
    positions = {}
    for item in items:
        try:
            aid_str, xy = item.split(":", 1)
            x_str, y_str = xy.split(",", 1)
            aid = int(aid_str)
            x = float(x_str)
            y = float(y_str)
        except ValueError as exc:
            raise ValueError(f"invalid --anchor '{item}', expected ID:X,Y") from exc
        positions[aid] = (x, y)
    return positions


# --------------------------------------------------------------------------
# Shared CSV writer (protected by asyncio — single-threaded event loop)
# --------------------------------------------------------------------------

csv_file   = None
csv_writer = None


def log_row(row: list):
    if csv_writer:
        csv_writer.writerow(row)
        csv_file.flush()


def cleanup_blink_groups(now_sec: float):
    expired = []
    for key, group in blink_groups.items():
        if now_sec - group["t0"] > BLINK_GROUP_TTL_SEC:
            expired.append(key)
    for key in expired:
        del blink_groups[key]


def solve_tdoa_2d(ref_id: int, ref_xy, obs):
    # obs: list of (anchor_id, (x,y), delta_range_m), where delta_range_m = ri-rref
    x = sum(pos[0] for _, pos, _ in obs) / len(obs)
    y = sum(pos[1] for _, pos, _ in obs) / len(obs)

    xr, yr = ref_xy
    max_iter = 10
    eps = 1e-9

    for _ in range(max_iter):
        r_ref = math.hypot(x - xr, y - yr)
        if r_ref < eps:
            r_ref = eps

        h11 = 0.0
        h12 = 0.0
        h22 = 0.0
        g1 = 0.0
        g2 = 0.0
        rss = 0.0

        for _, (xi, yi), delta_m in obs:
            ri = math.hypot(x - xi, y - yi)
            if ri < eps:
                ri = eps

            # f = model - measurement = (ri-rref)-delta
            f = (ri - r_ref) - delta_m
            rss += f * f

            dfx = (x - xi) / ri - (x - xr) / r_ref
            dfy = (y - yi) / ri - (y - yr) / r_ref

            h11 += dfx * dfx
            h12 += dfx * dfy
            h22 += dfy * dfy
            g1 += dfx * f
            g2 += dfy * f

        det = h11 * h22 - h12 * h12
        if abs(det) < 1e-12:
            return None, None, "singular geometry"

        # Solve H*step = -g
        step_x = (-g1 * h22 + g2 * h12) / det
        step_y = (g1 * h12 - g2 * h11) / det

        x += step_x
        y += step_y

        if step_x * step_x + step_y * step_y < 1e-8:
            rms = math.sqrt(rss / max(len(obs), 1))
            return x, y, rms

    # Return last iterate even if not fully converged.
    rms = math.sqrt(rss / max(len(obs), 1))
    return x, y, rms


def update_tdoa(ts: str, sync_seq: int, blink_seq: int, anchor_id: int, corrected: float):
    now_sec = time.time()
    cleanup_blink_groups(now_sec)

    key = (sync_seq, blink_seq)
    group = blink_groups.get(key)
    if group is None:
        group = {"t0": now_sec, "anchors": {}, "reported": 0}
        blink_groups[key] = group

    group["anchors"][anchor_id] = corrected
    anchors = group["anchors"]
    count = len(anchors)

    if count < 2 or count == group["reported"]:
        return

    ref_id = min(anchors.keys())
    ref_ts = anchors[ref_id]

    deltas = []
    for aid in sorted(anchors.keys()):
        if aid == ref_id:
            continue
        dt_ticks = anchors[aid] - ref_ts
        dt_ns = dt_ticks * DWT_TIME_UNIT_S * 1e9
        deltas.append((aid, dt_ticks, dt_ns))

    delta_str = "  ".join(
        f"a{aid}={dt_ticks:+.0f} ticks ({dt_ns:+.3f} ns)"
        for aid, dt_ticks, dt_ns in deltas
    )
    print(f"[{ts}] [TDOA] sync={sync_seq:3d} blink={blink_seq:3d} ref=a{ref_id}  {delta_str}")

    for aid, dt_ticks, dt_ns in deltas:
        log_row([
            ts, "", "TDOA", aid, blink_seq, sync_seq,
            "", "", "", "", "", "",
            ref_id, f"{dt_ticks:.0f}", f"{dt_ns:.3f}",
            "", "", ""
        ])

    # Multilateration from 3+ anchors with known coordinates.
    known = [aid for aid in anchors.keys() if aid in anchor_positions]
    if len(known) >= 3 and ref_id in anchor_positions:
        obs = []
        for aid in sorted(known):
            if aid == ref_id:
                continue
            dt_ticks = anchors[aid] - ref_ts
            dt_s = dt_ticks * DWT_TIME_UNIT_S
            delta_m = dt_s * SPEED_OF_LIGHT_M_S
            obs.append((aid, anchor_positions[aid], delta_m))

        if len(obs) >= 2:
            x, y, err = solve_tdoa_2d(ref_id, anchor_positions[ref_id], obs)
            if x is not None:
                print(
                    f"[{ts}] [POS ] sync={sync_seq:3d} blink={blink_seq:3d}"
                    f"  x={x:.3f} m  y={y:.3f} m  rms={err:.4f} m"
                )
                log_row([
                    ts, "", "POS", "", blink_seq, sync_seq,
                    "", "", "", "", "", "",
                    ref_id, "", "", f"{x:.3f}", f"{y:.3f}", f"{err:.4f}"
                ])
            else:
                print(
                    f"[{ts}] [POS ] sync={sync_seq:3d} blink={blink_seq:3d}"
                    f"  solver skipped ({err})"
                )

    group["reported"] = count


# --------------------------------------------------------------------------
# Per-device handler
# --------------------------------------------------------------------------

async def handle_device(device, stop_event: asyncio.Event):
    addr  = device.address
    short = addr[-8:]   # last 8 chars of MAC for compact display

    buf = ""

    def on_notify(_handle, data: bytearray):
        nonlocal buf
        buf += data.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            ts     = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            parts  = line.split(",")

            if parts[0] == "SYNC":
                if len(parts) != 8:
                    print(f"[{ts}] [{short}] WARN unexpected SYNC: {line!r}")
                    continue
                try:
                    anchor_id = int(parts[1])
                    seq       = int(parts[2])
                    tx_ts     = int(parts[3])
                    rx_ts     = int(parts[4])
                    offset    = int(parts[5])
                    drift     = float(parts[6])
                    corrected = float(parts[7])
                except ValueError:
                    print(f"[{ts}] [{short}] WARN parse error: {line!r}")
                    continue

                print(
                    f"[{ts}] [{short}] SYNC  a={anchor_id:3d}  seq={seq:4d}"
                    f"  tx={tx_ts}  rx={rx_ts}"
                    f"  offset={offset:+d}  drift={drift:.9f}"
                    f"  corrected={corrected:.0f}"
                )
                log_row([
                    ts, addr, "SYNC", anchor_id, seq, "",
                    tx_ts, rx_ts, offset, drift, corrected, "",
                    "", "", ""
                ])

            elif parts[0] == "BLINK":
                if len(parts) != 6:
                    print(f"[{ts}] [{short}] WARN unexpected BLINK: {line!r}")
                    continue
                try:
                    anchor_id   = int(parts[1])
                    blink_seq   = int(parts[2])
                    sync_seq    = int(parts[3])
                    sync_tx_ts  = int(parts[4])
                    master_time = float(parts[5])
                except ValueError:
                    print(f"[{ts}] [{short}] WARN parse error: {line!r}")
                    continue

                print(
                    f"[{ts}] [{short}] BLINK a={anchor_id:3d}"
                    f"  bseq={blink_seq:3d}  sseq={sync_seq:3d}"
                    f"  sync_tx={sync_tx_ts}  master_time={master_time:.0f}"
                )
                log_row([
                    ts, addr, "BLINK", anchor_id, blink_seq, sync_seq,
                    sync_tx_ts, "", "", "", "", master_time,
                    "", "", ""
                ])
                update_tdoa(ts, sync_seq, blink_seq, anchor_id, master_time)

            else:
                print(f"[{ts}] [{short}] WARN unknown: {line!r}")

    while not stop_event.is_set():
        try:
            print(f"  Connecting to [{addr}] ...")
            async with BleakClient(device, timeout=10.0) as client:
                print(f"  Connected  [{addr}]  MTU={client.mtu_size}")
                await client.start_notify(NUS_TX_UUID, on_notify)
                # Stay connected until stop or disconnection exception
                while not stop_event.is_set() and client.is_connected:
                    await asyncio.sleep(0.5)
                await client.stop_notify(NUS_TX_UUID)
        except Exception as exc:
            if stop_event.is_set():
                break
            print(f"  [{short}] disconnected ({exc}), retrying in {RECONNECT_SEC}s ...")
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=RECONNECT_SEC)
            except asyncio.TimeoutError:
                pass

    print(f"  [{short}] handler exited")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

async def main(scan_time: float, log_path: str):
    global csv_file, csv_writer

    if anchor_positions:
        print("Anchor geometry (id -> x,y meters):")
        for aid in sorted(anchor_positions.keys()):
            x, y = anchor_positions[aid]
            print(f"  a{aid}: ({x:.3f}, {y:.3f})")
    else:
        print("No --anchor positions provided; only TDoA deltas will be reported.")

    if log_path:
        csv_file   = open(log_path, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow([
                "time", "addr", "type", "anchor_id", "seq", "sync_seq",
                "tx_ts", "rx_ts", "offset", "drift", "corrected", "master_time",
                "ref_anchor", "delta_ticks", "delta_ns", "x_m", "y_m", "rms_m"
            ])
        print(f"Logging to {log_path}")

    # ---- Scan ----
    print(f"Scanning {scan_time}s for '{DEVICE_NAME}' devices ...")
    found = {}

    def detection_cb(device, adv_data):
        name = adv_data.local_name or device.name or ""
        if name == DEVICE_NAME and device.address not in found:
            found[device.address] = device
            print(f"  Found: [{device.address}]  RSSI={adv_data.rssi} dBm")

    async with BleakScanner(detection_callback=detection_cb):
        await asyncio.sleep(scan_time)

    if not found:
        print(f"No '{DEVICE_NAME}' devices found. Is the firmware flashed and advertising?")
        sys.exit(1)

    print(f"\nFound {len(found)} device(s). Connecting to all ...\n")

    stop_event = asyncio.Event()

    tasks = [
        asyncio.create_task(handle_device(dev, stop_event))
        for dev in found.values()
    ]

    try:
        await asyncio.gather(*tasks)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        # Give tasks a moment to exit cleanly
        await asyncio.gather(*tasks, return_exceptions=True)

    if csv_file:
        csv_file.close()

    print("Done.")


def entry():
    args = parse_args()
    global anchor_positions
    try:
        anchor_positions = parse_anchor_positions(args.anchor)
    except ValueError as exc:
        print(f"Argument error: {exc}")
        sys.exit(2)
    try:
        asyncio.run(main(args.scan_time, args.log))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    entry()
