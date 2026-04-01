import asyncio
from bleak import BleakClient, BleakScanner
from collections import defaultdict
import numpy as np
from scipy.optimize import least_squares

DEVICE_NAME = "DWM3001-TDOA"
UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

C = 299792458.0
DTU_TO_SEC = 15.65e-12

ANCHORS = {
    6: (0.0, 1.04),
    7: (0.0, 0.0),
    10: (0.5, 0.5),
}

data_store = defaultdict(dict)

def tdoa_residuals(pos, anchor_positions, tdoa_meters):
    x, y = pos
    ref_pos = anchor_positions[0]
    d_ref = np.sqrt((x - ref_pos[0])**2 + (y - ref_pos[1])**2)

    residuals = []
    for i in range(len(tdoa_meters)):
        ai = anchor_positions[i + 1]
        d_i = np.sqrt((x - ai[0])**2 + (y - ai[1])**2)
        residuals.append((d_i - d_ref) - tdoa_meters[i])

    return residuals

def compute_position(seq_data):
    if len(seq_data) < 3:
        return None

    ids = sorted(seq_data.keys())
    ref_id = ids[0]
    t_ref = seq_data[ref_id]

    anchor_positions = [ANCHORS[ref_id]]
    tdoa_meters = []

    for i in range(1, len(ids)):
        aid = ids[i]
        dt = seq_data[aid] - t_ref
        dd = C * dt * DTU_TO_SEC
        anchor_positions.append(ANCHORS[aid])
        tdoa_meters.append(dd)

    x0 = np.mean([p[0] for p in anchor_positions])
    y0 = np.mean([p[1] for p in anchor_positions])

    result = least_squares(
        tdoa_residuals, [x0, y0],
        args=(anchor_positions, tdoa_meters)
    )

    if result.success:
        return (result.x[0], result.x[1])
    return None

# ------------------------------------------------------------------

def process_packet(node_id, seq, master_time):

    data_store[seq][node_id] = master_time

    # Wait until all 3 anchors received same seq
    if len(data_store[seq]) >= 3:

        seq_data = data_store[seq]

        print(f"\n=== SEQ {seq} ===")

        for nid, t in seq_data.items():
            print(f"Anchor {nid}: {t}")

        # Compute Δt for debug
        ids = sorted(seq_data.keys())
        t0 = seq_data[ids[0]]

        for i in range(1, len(ids)):
            dt = seq_data[ids[i]] - t0
            dd = C * dt * DTU_TO_SEC
            print(f"Δt({ids[i]}-{ids[0]}) = {dt} DTU → {dd:.3f} m")

        # Cleanup
        del data_store[seq]

# ------------------------------------------------------------------

def make_callback():
    def callback(_, data: bytearray):
        try:
            line = data.decode().strip()
            parts = line.split(",")

            if parts[0] == "BLINK":
                node_id = int(parts[1])
                seq = int(parts[2])
                master_time = float(parts[4])

                process_packet(node_id, seq, master_time)

        except Exception as e:
            print("Parse error:", e)

    return callback

# ------------------------------------------------------------------

async def connect_device(device):

    while True:
        try:
            async with BleakClient(device.address) as client:
                print(f"Connected: {device.address}")

                await client.start_notify(UUID, make_callback())

                while True:
                    await asyncio.sleep(1)

        except Exception as e:
            print(f"Reconnect {device.address}: {e}")
            await asyncio.sleep(2)

# ------------------------------------------------------------------

async def main():

    print("Scanning for anchors...")
    devices = await BleakScanner.discover(timeout=15)

    targets = []

    for d in devices:
        if d.name == DEVICE_NAME:
            print(f"Found: {d.address}")
            targets.append(d)

    if len(targets) < 3:
        print("❌ Need at least 3 anchors")
        return

    await asyncio.gather(*(connect_device(d) for d in targets))

# ------------------------------------------------------------------

asyncio.run(main())