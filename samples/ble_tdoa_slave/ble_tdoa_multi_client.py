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
"""

import asyncio
import argparse
import csv
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME   = "DWM3001-TDOA"
NUS_TX_UUID   = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
RECONNECT_SEC = 5          # seconds to wait before reconnect attempt


def parse_args():
    p = argparse.ArgumentParser(description="Connect to multiple DWM3001-TDOA slaves over BLE")
    p.add_argument("--scan-time", type=float, default=5.0,
                   metavar="SEC", help="Initial scan duration in seconds (default: 5)")
    p.add_argument("--log", metavar="FILE", help="CSV file to log all data (appended)")
    return p.parse_args()


# --------------------------------------------------------------------------
# Shared CSV writer (protected by asyncio — single-threaded event loop)
# --------------------------------------------------------------------------

csv_file   = None
csv_writer = None


def log_row(row: list):
    if csv_writer:
        csv_writer.writerow(row)
        csv_file.flush()


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
                if len(parts) != 7:
                    print(f"[{ts}] [{short}] WARN unexpected SYNC: {line!r}")
                    continue
                try:
                    seq       = int(parts[1])
                    tx_ts     = int(parts[2])
                    rx_ts     = int(parts[3])
                    offset    = int(parts[4])
                    drift     = float(parts[5])
                    corrected = float(parts[6])
                except ValueError:
                    print(f"[{ts}] [{short}] WARN parse error: {line!r}")
                    continue

                print(
                    f"[{ts}] [{short}] SYNC  seq={seq:4d}"
                    f"  tx={tx_ts}  rx={rx_ts}"
                    f"  offset={offset:+d}  drift={drift:.9f}"
                    f"  corrected={corrected:.0f}"
                )
                log_row([ts, addr, "SYNC", seq, tx_ts, rx_ts,
                         offset, drift, corrected, ""])

            elif parts[0] == "BLINK":
                if len(parts) != 3:
                    print(f"[{ts}] [{short}] WARN unexpected BLINK: {line!r}")
                    continue
                try:
                    rx_ts       = int(parts[1])
                    master_time = float(parts[2])
                except ValueError:
                    print(f"[{ts}] [{short}] WARN parse error: {line!r}")
                    continue

                print(
                    f"[{ts}] [{short}] BLINK"
                    f"  rx={rx_ts}  master_time={master_time:.0f}"
                )
                log_row([ts, addr, "BLINK", "", "", rx_ts,
                         "", "", "", master_time])

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

    if log_path:
        csv_file   = open(log_path, "a", newline="")
        csv_writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            csv_writer.writerow(["time", "addr", "type", "seq",
                                 "tx_ts", "rx_ts", "offset",
                                 "drift", "corrected", "master_time"])
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
    try:
        asyncio.run(main(args.scan_time, args.log))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    entry()
