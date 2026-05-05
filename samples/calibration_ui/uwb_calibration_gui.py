import serial
import threading
import numpy as np
import tkinter as tk

BAUD = 115200
TRUE_DISTANCE = 0.50
SPEED_OF_LIGHT = 299702547
DWT_TIME_UNIT = 1/(499.2e6*128)

ser = None
running = False
samples = []
delay = 26194

def connect():

    global ser

    port = port_entry.get()

    try:
        ser = serial.Serial(port, BAUD, timeout=1)
        log("Connected to " + port)
        status_label.config(text="CONNECTED", fg="#00ffa6")

    except Exception as e:
        log("Connection failed: " + str(e))
        status_label.config(text="NOT CONNECTED", fg="red")

def log(msg):

    log_box.insert(tk.END, msg + "\n")
    log_box.see(tk.END)

def read_serial():

    global running

    while running:

        try:

            line = ser.readline().decode(errors="ignore").strip()

            if line.startswith("DIST"):

                dist = float(line.split()[1])

                samples.append(dist)

                dist_var.set(f"{dist:.3f} m")

        except:
            pass

def start_sampling():

    global running
    global samples

    if ser is None:
        log("Connect serial first")
        return

    samples = []
    running = True

    threading.Thread(target=read_serial, daemon=True).start()

    animate()

    log("Sampling started")

def stop_sampling():

    global running

    running = False
    log("Sampling stopped")

def calibrate():

    global delay

    if len(samples) < 10:
        log("Not enough samples")
        return

    avg = np.mean(samples)

    avg_var.set(f"{avg:.3f} m")

    error = avg - TRUE_DISTANCE

    time_error = error / SPEED_OF_LIGHT

    correction = int(time_error / DWT_TIME_UNIT)

    delay -= correction

    try:
        ser.write(f"SET_DELAY {delay}\n".encode())
    except:
        log("Failed to send delay")

    delay_var.set(str(delay))

    log(f"New antenna delay = {delay}")

def animate():

    if running:

        dots = indicator_var.get()

        if len(dots) > 3:
            dots = ""

        indicator_var.set(dots + ".")

        root.after(400, animate)

root = tk.Tk()
root.title("UWB Calibration Tool")
root.geometry("900x650")
root.configure(bg="#0b0b0b")

title = tk.Label(
    root,
    text="Society of Robotics and Automation",
    font=("Segoe UI", 20, "bold"),
    fg="white",
    bg="#0b0b0b"
)

title.pack(pady=(10,4))

subtitle = tk.Label(
    root,
    text="UWB Calibration Tool",
    font=("Segoe UI", 16),
    fg="#3da9fc",
    bg="#0b0b0b"
)

subtitle.pack()

instruction = tk.Label(
    root,
    text="Place both devices exactly 50 cm apart for calibration",
    font=("Segoe UI", 13),
    fg="#00ffa6",
    bg="#0b0b0b"
)

instruction.pack(pady=5)

canvas = tk.Canvas(root, width=620, height=70, bg="#0b0b0b", highlightthickness=0)
canvas.pack()

canvas.create_rectangle(70,30,100,55,outline="#00ffa6",width=2)
canvas.create_rectangle(520,30,550,55,outline="#00ffa6",width=2)

canvas.create_line(100,42,520,42,fill="white",width=2)

canvas.create_text(310,20,text="50 cm",fill="white",font=("Segoe UI",10))

canvas.create_text(85,65,text="Device A",fill="white",font=("Segoe UI",9))
canvas.create_text(535,65,text="Device B",fill="white",font=("Segoe UI",9))

frame = tk.Frame(root, bg="#0b0b0b")
frame.pack(pady=15)

port_label = tk.Label(frame, text="COM Port", fg="white", bg="#0b0b0b", font=("Segoe UI",11))
port_label.grid(row=0, column=0, padx=12)

port_entry = tk.Entry(frame, width=9, font=("Segoe UI",11))
port_entry.insert(0, "COM12")
port_entry.grid(row=0, column=1, padx=8)

connect_btn = tk.Button(
    frame,
    text="Connect",
    command=connect,
    bg="#3da9fc",
    fg="black",
    font=("Segoe UI",10,"bold"),
    width=9
)

connect_btn.grid(row=0, column=2, padx=8)

status_label = tk.Label(frame, text="NOT CONNECTED", fg="red", bg="#0b0b0b", font=("Segoe UI",10))
status_label.grid(row=0, column=3, padx=10)

start_btn = tk.Button(
    frame,
    text="Start Sampling",
    command=start_sampling,
    bg="#00ffa6",
    fg="black",
    font=("Segoe UI",10,"bold"),
    width=13
)

start_btn.grid(row=1, column=0, pady=12)

stop_btn = tk.Button(
    frame,
    text="Stop Sampling",
    command=stop_sampling,
    bg="#ff5959",
    fg="white",
    font=("Segoe UI",10,"bold"),
    width=13
)

stop_btn.grid(row=1, column=1)

cal_btn = tk.Button(
    frame,
    text="Calibrate",
    command=calibrate,
    bg="#3da9fc",
    fg="black",
    font=("Segoe UI",10,"bold"),
    width=13
)

cal_btn.grid(row=1, column=2)

dist_var = tk.StringVar(value="0.000 m")
avg_var = tk.StringVar(value="0.000 m")
delay_var = tk.StringVar(value=str(delay))
indicator_var = tk.StringVar(value="")

live_frame = tk.Frame(root, bg="#0b0b0b")
live_frame.pack(pady=18)

tk.Label(live_frame, text="Current Distance", fg="white", bg="#0b0b0b", font=("Segoe UI",13)).grid(row=0, column=0, padx=35)
tk.Label(live_frame, textvariable=dist_var, fg="#00ffa6", bg="#0b0b0b", font=("Segoe UI",15,"bold")).grid(row=0, column=1)

tk.Label(live_frame, text="Average Distance", fg="white", bg="#0b0b0b", font=("Segoe UI",13)).grid(row=1, column=0)
tk.Label(live_frame, textvariable=avg_var, fg="#00ffa6", bg="#0b0b0b", font=("Segoe UI",15,"bold")).grid(row=1, column=1)

tk.Label(live_frame, text="Antenna Delay", fg="white", bg="#0b0b0b", font=("Segoe UI",13)).grid(row=2, column=0)
tk.Label(live_frame, textvariable=delay_var, fg="#00ffa6", bg="#0b0b0b", font=("Segoe UI",15,"bold")).grid(row=2, column=1)

tk.Label(live_frame, textvariable=indicator_var, fg="#00ffa6", bg="#0b0b0b", font=("Segoe UI",14)).grid(row=3, column=1)

log_box = tk.Text(
    root,
    height=7,
    width=95,
    bg="#101820",
    fg="white",
    font=("Consolas",9),
    insertbackground="white"
)

log_box.pack(pady=15)

root.mainloop()