import tkinter as tk
from tkinter import ttk
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import serial.tools.list_ports
import time

def start_plot():
    port = port_select.get()
    y_min = float(ymin_entry.get())
    y_max = float(ymax_entry.get())
    keep_points = int(points_entry.get())

    show_sens = sens_var.get()
    show_ins = ins_var.get()
    show_keya = keya_var.get()
    show_kalman = kalman_var.get()

    root.destroy()

    ser = serial.Serial(port, 115200, timeout=1)
    steer_vals, ins_vals, keya_vals, kalman_vals = [], [], [], []

    fig, ax = plt.subplots()
    ln_sens, = ax.plot([], [], 'r-', label="sens") if show_sens else (None,)
    ln_ins, = ax.plot([], [], 'g-', label="insAngle") if show_ins else (None,)
    ln_keya, = ax.plot([], [], 'b-', label="keyaAngle") if show_keya else (None,)
    ln_kalman, = ax.plot([], [], 'm-', label="Kalman") if show_kalman else (None,)
    plt.legend()

    def init():
        ax.set_xlim(0, keep_points)
        ax.set_ylim(y_min, y_max)
        return [ln for ln in [ln_sens, ln_ins, ln_keya, ln_kalman] if ln]

    def update():
        line = ser.readline().decode('utf-8').strip()
        match = re.findall(r"sens:(-?\d+\.?\d*),insAngle:(-?\d+\.?\d*),keyaAngle:(-?\d+\.?\d*),Kalman:(-?\d+\.?\d*)", line)
        if match:
            s, i, k, m = map(float, match[0])
            if show_sens:
                steer_vals.append(s)
                steer_vals[:] = steer_vals[-keep_points:]
                ln_sens.set_data(range(len(steer_vals)), steer_vals)
            if show_ins:
                ins_vals.append(i)
                ins_vals[:] = ins_vals[-keep_points:]
                ln_ins.set_data(range(len(ins_vals)), ins_vals)
            if show_keya:
                keya_vals.append(k)
                keya_vals[:] = keya_vals[-keep_points:]
                ln_keya.set_data(range(len(keya_vals)), keya_vals)
            if show_kalman:
                kalman_vals.append(m)
                kalman_vals[:] = kalman_vals[-keep_points:]
                ln_kalman.set_data(range(len(kalman_vals)), kalman_vals)
            ax.set_xlim(0, keep_points)
            ax.figure.canvas.draw()
            ax.set_ylim(y_min, y_max)
        root.after(10, update)

    ser.flushInput()
    root.after(10, update)
    plt.show()

root = tk.Tk()
root.title("Serial Plotter Setup")

# Get the list of available serial ports
available_ports = [port.device for port in serial.tools.list_ports.comports()]

tk.Label(root, text="COM Port (Windows):").grid(row=0, column=0, padx=5, pady=5)
port_select = ttk.Combobox(root, values=available_ports)
if available_ports:
    port_select.current(0)
port_select.grid(row=0, column=1, padx=5, pady=5)

tk.Label(root, text="Y-axis Min:").grid(row=1, column=0, padx=5, pady=5)
ymin_entry = ttk.Entry(root)
ymin_entry.insert(0, "-50")
ymin_entry.grid(row=1, column=1, padx=5, pady=5)

tk.Label(root, text="Y-axis Max:").grid(row=2, column=0, padx=5, pady=5)
ymax_entry = ttk.Entry(root)
ymax_entry.insert(0, "50")
ymax_entry.grid(row=2, column=1, padx=5, pady=5)

tk.Label(root, text="Points to Keep:").grid(row=3, column=0, padx=5, pady=5)
points_entry = ttk.Entry(root)
points_entry.insert(0, "100")
points_entry.grid(row=3, column=1, padx=5, pady=5)

sens_var = tk.BooleanVar(value=True)
ins_var = tk.BooleanVar(value=True)
keya_var = tk.BooleanVar(value=True)
kalman_var = tk.BooleanVar(value=True)

tk.Checkbutton(root, text="Show sens", variable=sens_var).grid(row=4, column=0, padx=5, pady=5)
tk.Checkbutton(root, text="Show insAngle", variable=ins_var).grid(row=4, column=1, padx=5, pady=5)
tk.Checkbutton(root, text="Show keyaAngle", variable=keya_var).grid(row=5, column=0, padx=5, pady=5)
tk.Checkbutton(root, text="Show Kalman", variable=kalman_var).grid(row=5, column=1, padx=5, pady=5)

btn_start = ttk.Button(root, text="Start Plot", command=start_plot)
btn_start.grid(row=6, column=0, columnspan=2, padx=5, pady=5)

root.mainloop()