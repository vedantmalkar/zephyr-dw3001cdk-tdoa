#!/usr/bin/env python3
"""
BLE TDoA Slave Client

Connects to DWM3001-TDOA (ble_tdoa_slave firmware), subscribes to
NUS TX notifications, and prints the two CSV formats:

  SYNC frames:  "SYNC,seq,tx_ts,rx_ts,offset,drift,corrected"
  BLINK frames: "BLINK,rx_ts,master_time"

Usage:
  python ble_tdoa_slave_client.py [--log <file.csv>]
"""

import asyncio
import argparse
import csv
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "DWM3001-TDOA"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

def parse_args():
    p = argparse.ArgumentParser(description="BLE TDoA slave receiver")
    p.add_argument("--log", metavar="FILE", help="CSV file to log data (appended)")
    return p.parse_args()

async def run(log_path):
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if device is None:
        print(f"ERROR: '{DEVICE_NAME}' not found. Is the board advertising?")
        sys.exit(1)

    print(f"Found {device.name} [{device.address}]")

    csv_file = None
    writer = None
    if log_path:
        csv_file = open(log_path, "a", newline="")
        writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            writer.writerow(["type", "seq", "tx_ts", "rx_ts",
                             "offset", "drift", "corrected", "master_time"])
        print(f"Logging to {log_path}")

    buf = ""

    def on_notify(_handle, data: bytearray):
        nonlocal buf
        buf += data.decode("utf-8", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            parts = line.split(",")

            if parts[0] == "SYNC":
                if len(parts) != 7:
                    print(f"[WARN] unexpected SYNC format: {line!r}")
                    continue
                try:
                    seq       = int(parts[1])
                    tx_ts     = int(parts[2])
                    rx_ts     = int(parts[3])
                    offset    = int(parts[4])
                    drift     = float(parts[5])
                    corrected = float(parts[6])
                except ValueError:
                    print(f"[WARN] parse error: {line!r}")
                    continue

                print(
                    f"[{ts}] SYNC  seq={seq:4d}  tx={tx_ts}  rx={rx_ts}"
                    f"  offset={offset:+d}  drift={drift:.9f}  corrected={corrected:.0f}"
                )

                if writer:
                    writer.writerow(["SYNC", seq, tx_ts, rx_ts,
                                     offset, drift, corrected, ""])
                    csv_file.flush()

            elif parts[0] == "BLINK":
                if len(parts) != 3:
                    print(f"[WARN] unexpected BLINK format: {line!r}")
                    continue
                try:
                    rx_ts       = int(parts[1])
                    master_time = float(parts[2])
                except ValueError:
                    print(f"[WARN] parse error: {line!r}")
                    continue

                print(
                    f"[{ts}] BLINK rx={rx_ts}  master_time={master_time:.0f}"
                )

                if writer:
                    writer.writerow(["BLINK", "", "", rx_ts,
                                     "", "", "", master_time])
                    csv_file.flush()

            else:
                print(f"[WARN] unknown message type: {line!r}")

    async with BleakClient(device) as client:
        print(f"Connected (MTU={client.mtu_size})")
        await client.start_notify(NUS_TX_UUID, on_notify)
        print(f"Subscribed to NUS TX. Receiving TDoA data - press Ctrl+C to stop.\n")
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            await client.stop_notify(NUS_TX_UUID)

    if csv_file:
        csv_file.close()

def main():
    args = parse_args()
    asyncio.run(run(args.log))

if __name__ == "__main__":
    main()
