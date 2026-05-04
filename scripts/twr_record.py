#!/usr/bin/env python3
"""
DS-TWR Distance Recorder

Reads distance measurements from the tag over serial
and saves them to a JSON file for later analysis.

Usage:
  python3 twr_record.py --port /dev/ttyACM0
  python3 twr_record.py --port /dev/ttyACM0 -o my_recording.json

Press Ctrl+C to stop recording and save.
"""

import serial
import re
import json
import time
import argparse
from datetime import datetime

PORT = "/dev/ttyACM0"
BAUD = 115200

# Anchor positions in meters (x, y)
ANCHORS = {
    1: [0.90, 0.60],
    3: [0.90, 0.00],
    6: [0.00, 0.60],
    7: [0.00, 0.00],
}

PATTERN = re.compile(
    r"Anchor\s+(\d+):\s+(-?\d+\.?\d*)\s+m\s+seq=(\d+)"
)


def main():
    parser = argparse.ArgumentParser(description="Record DS-TWR distances to JSON")
    parser.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument("--baud", type=int, default=BAUD, help=f"Baud rate (default: {BAUD})")
    parser.add_argument("-o", "--output", default=None, help="Output filename (default: twr_TIMESTAMP.json)")
    args = parser.parse_args()

    if args.output is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"twr_{ts}.json"

    print("=" * 60)
    print("DS-TWR Distance Recorder")
    print("=" * 60)
    print(f"Serial: {args.port} @ {args.baud}")
    print(f"Output: {args.output}")
    print(f"Anchors: {list(ANCHORS.keys())}")
    print("Press Ctrl+C to stop and save.")
    print("=" * 60)
    print()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()

    samples = []
    current_round = {}
    last_anchor = None
    sample_count = 0
    start_time = datetime.now()

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode(errors="ignore").strip()
            if not line:
                continue

            m = PATTERN.search(line)
            if not m:
                continue

            anchor_id = int(m.group(1))
            dist = float(m.group(2))

            # if we see an anchor we already have, the sweep is complete
            if anchor_id in current_round and len(current_round) >= 2:
                samples.append({
                    "sample": sample_count,
                    "timestamp": datetime.now().isoformat(),
                    "distances": dict(current_round),
                })
                sample_count += 1
                if sample_count % 10 == 0:
                    print(f"  [{sample_count} samples saved]")
                current_round = {}

            current_round[anchor_id] = dist

            print(f"[{sample_count:05d}] A{anchor_id}: {dist:.2f} m")

    except KeyboardInterrupt:
        if current_round:
            samples.append({
                "sample": sample_count,
                "timestamp": datetime.now().isoformat(),
                "distances": dict(current_round),
            })
            sample_count += 1

    finally:
        ser.close()

    end_time = datetime.now()
    duration = (end_time - start_time).total_seconds()

    output = {
        "session_info": {
            "start_time": start_time.isoformat(),
            "end_time": end_time.isoformat(),
            "duration_seconds": round(duration, 1),
            "total_samples": sample_count,
            "port": args.port,
        },
        "anchor_positions": {str(k): v for k, v in ANCHORS.items()},
        "samples": samples,
    }

    with open(args.output, "w") as f:
        json.dump(output, f, indent=2)

    print(f"\n\nSaved {sample_count} samples to {args.output}")
    print(f"Duration: {duration:.1f}s")


if __name__ == "__main__":
    main()
