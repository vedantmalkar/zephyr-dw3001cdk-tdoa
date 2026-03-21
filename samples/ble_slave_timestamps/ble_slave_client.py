#!/usr/bin/env python3
"""
BLE Slave Timestamp Client

Connects to DWM3001-SLAVE (ble_slave_timestamps firmware), subscribes to
NUS TX notifications, and prints the CSV stream:
  seq, tx_ts, rx_ts, offset, drift, corrected

Usage:
  python ble_slave_client.py [--log <file.csv>]
"""

import asyncio
import argparse
import csv
import sys
from datetime import datetime

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "DWM3001-SLAVE"

NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

HEADER = ["seq", "tx_ts", "rx_ts", "offset", "drift", "corrected"]


def parse_args():
    p = argparse.ArgumentParser(description="BLE slave timestamp receiver")
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
        # Write header only if file is empty
        if csv_file.tell() == 0:
            writer.writerow(HEADER)
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
            parts = line.split(",")
            if len(parts) != 6:
                print(f"[WARN] unexpected format: {line!r}")
                continue
            try:
                seq       = int(parts[0])
                tx_ts     = int(parts[1])
                rx_ts     = int(parts[2])
                offset    = int(parts[3])
                drift     = float(parts[4])
                corrected = float(parts[5])
            except ValueError:
                print(f"[WARN] parse error: {line!r}")
                continue

            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(
                f"[{ts}] seq={seq:4d}  tx={tx_ts}  rx={rx_ts}"
                f"  offset={offset:+d}  drift={drift:.9f}  corrected={corrected:.0f}"
            )

            if writer:
                writer.writerow([seq, tx_ts, rx_ts, offset, drift, corrected])
                csv_file.flush()

    async with BleakClient(device) as client:
        print(f"Connected (MTU={client.mtu_size})")
        await client.start_notify(NUS_TX_UUID, on_notify)
        print(f"Subscribed to NUS TX. Receiving timestamps — press Ctrl+C to stop.\n")
        print(f"{'seq':>6}  {'tx_ts':>20}  {'rx_ts':>20}  {'offset':>12}  {'drift':>14}  {'corrected':>20}")
        print("-" * 100)
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
