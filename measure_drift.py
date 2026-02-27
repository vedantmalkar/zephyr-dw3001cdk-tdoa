import serial
import time

# --- CONFIG ---
PORT_A = '/dev/ttyACM0'   # first chip
PORT_B = '/dev/ttyACM1'   # second chip
BAUD = 115200
WAIT_SECONDS = 5
# DW3000 timestamp clock: 499.2 MHz * 128 = 63.8976 GHz
DWT_FREQ = 499.2e6 * 128
# 40-bit counter rollover mask
MASK_40 = (1 << 40) - 1
# --------------

def read_timestamp(ser):
    """Send 'r', wait for a line starting with 'TS:', return integer tick count."""
    ser.write(b'r')
    for _ in range(30):  # skip log lines, wait for TS response
        line = ser.readline().decode(errors='ignore').strip()
        if line.startswith('TS:'):
            return int(line[3:], 16)
    raise TimeoutError(f"No timestamp on {ser.port}")

def main():
    print(f"Opening {PORT_A} and {PORT_B} at {BAUD} baud...")
    with serial.Serial(PORT_A, BAUD, timeout=2) as a, \
         serial.Serial(PORT_B, BAUD, timeout=2) as b:

        a.reset_input_buffer()
        b.reset_input_buffer()
        time.sleep(0.2)

        # --- First read: A then B (same order both times so offset cancels) ---
        ts_a1 = read_timestamp(a)
        ts_b1 = read_timestamp(b)
        t_wall_0 = time.monotonic()
        print(f"[t=0]   Chip A: {ts_a1:#012x}  Chip B: {ts_b1:#012x}")

        time.sleep(WAIT_SECONDS)

        # --- Second read: A then B ---
        ts_a2 = read_timestamp(a)
        ts_b2 = read_timestamp(b)
        t_wall_1 = time.monotonic()
        print(f"[t=5s]  Chip A: {ts_a2:#012x}  Chip B: {ts_b2:#012x}")

        # --- Compute periods (handle 40-bit rollover) ---
        period_a = (ts_a2 - ts_a1) & MASK_40
        period_b = (ts_b2 - ts_b1) & MASK_40
        wall_elapsed = t_wall_1 - t_wall_0

        # --- Drift ---
        drift_ticks = int(period_a) - int(period_b)
        drift_ppm   = drift_ticks / period_b * 1e6
        drift_us_s  = (drift_ticks / DWT_FREQ) / wall_elapsed * 1e6

        print()
        print(f"Wall clock elapsed : {wall_elapsed:.3f} s")
        print(f"Period A           : {period_a} ticks  ({period_a / DWT_FREQ:.6f} s)")
        print(f"Period B           : {period_b} ticks  ({period_b / DWT_FREQ:.6f} s)")
        print(f"Drift (ticks)      : {drift_ticks:+d}")
        print(f"Drift (ppm)        : {drift_ppm:+.2f} ppm")
        print(f"Drift (us/s)       : {drift_us_s:+.3f} us/s")

if __name__ == '__main__':
    main()
