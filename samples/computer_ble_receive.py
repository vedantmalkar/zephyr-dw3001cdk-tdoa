import asyncio
from bleak import BleakClient, BleakScanner
from collections import defaultdict
import math

DEVICE_NAME = "DWM3001-TDOA"
UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# Constants
C = 299792458                  # speed of light (m/s)
DTU_TO_SEC = 15.65e-12         # DW3000 time unit

# Anchor positions (meters)
ANCHORS = {
    6: (0.0, 0.0),
    7: (5.0, 0.0),
    10: (0.0, 5.0),
}

# seq → {anchor_id: time}
data_store = defaultdict(dict)

# ------------------------------------------------------------------

def compute_position(seq_data):
    """
    TDOA solver (linearized, 3 anchors)
    """

    if len(seq_data) < 3:
        return None

    ids = sorted(seq_data.keys())

    ref_id = ids[0]
    t_ref = seq_data[ref_id]
    x1, y1 = ANCHORS[ref_id]

    A = []
    B = []

    for i in range(1, len(ids)):
        aid = ids[i]
        t_i = seq_data[aid]
        x2, y2 = ANCHORS[aid]

        dt = (t_i - t_ref)   # in DTU
        dd = C * dt * DTU_TO_SEC  # meters

        A.append([
            2*(x2 - x1),
            2*(y2 - y1)
        ])

        B.append(
            (x2**2 + y2**2 - x1**2 - y1**2) - dd**2
        )

    try:
        A1, A2 = A[0], A[1]
        B1, B2 = B[0], B[1]

        det = A1[0]*A2[1] - A1[1]*A2[0]

        if abs(det) < 1e-6:
            return None

        x = (B1*A2[1] - B2*A1[1]) / det
        y = (A1[0]*B2 - A2[0]*B1) / det

        return (x, y)

    except:
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

        # Solve position
        pos = compute_position(seq_data)

        if pos:
            print(f"📍 Position: x={pos[0]:.2f} m, y={pos[1]:.2f} m")
        else:
            print("⚠️ Position solve failed")

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
    devices = await BleakScanner.discover()

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