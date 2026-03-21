#!/usr/bin/env python3
"""
BLE TX Timestamp client for DWM3001CDK.

Connects to the chip and prints every TX timestamp notification.
Each notification is a UTF-8 string: "seq=<n> ts=<timestamp>\n"

Install:
    pip install bleak

Run:
    python ble_tx_ts_client.py
    python ble_tx_ts_client.py --addr AA:BB:CC:DD:EE:FF
"""

import asyncio
import argparse
import sys
from datetime import datetime
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

DEVICE_NAME = "DWM3001-TXTS"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

def on_notification(characteristic: BleakGATTCharacteristic, data: bytearray):
    line = data.decode("utf-8", errors="replace").strip()
    wall = datetime.now().isoformat(timespec="milliseconds")
    print(f"[{wall}]  {line}")

async def main(name: str, addr: str | None) -> None:
    if addr:
        target = addr
    else:
        print(f"Scanning for '{name}' ...")
        device = await BleakScanner.find_device_by_name(name, timeout=10.0)
        if device is None:
            print(f"ERROR: '{name}' not found. Is the chip advertising?")
            sys.exit(1)
        print(f"Found: {device.name}  [{device.address}]")
        target = device.address

    print(f"Connecting to {target} ...")
    async with BleakClient(target) as client:
        print("Connected. Receiving TX timestamps — Ctrl-C to stop.\n")
        await client.start_notify(NUS_TX_UUID, on_notification)
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(NUS_TX_UUID)

    print("Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BLE TX timestamp client for DWM3001CDK")
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--addr", default=None)
    args = parser.parse_args()
    asyncio.run(main(args.name, args.addr))
