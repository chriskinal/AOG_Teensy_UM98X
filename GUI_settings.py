import socket
import tkinter as tk
from tkinter import ttk
import json

# ====== NETWORK SETTINGS ======
SERVER_IP = '192.168.5.120'  # Change to your Arduino's IP
SERVER_PORT = 8888           # Change to your UDP port

# ====== HEADER DEFINITION ======
HEADER = bytes([0x80, 0x81, 0x7F, 0x69])  # 4-byte header

# ====== FUNCTION TO SEND UDP PACKET ======
def send_packet():
    # Get the float values from the input fields
    try:
        values = [float(entry.get()) for entry in entry_fields]
    except ValueError:
        status_label.config(text="Error: Please enter valid float numbers.", fg="red")
        return

    # Format the float values as a comma-separated string
    data_str = ",".join(f"{value}" for value in values)
    
    # Combine the header (binary) with the string (encoded as bytes)
    packet = HEADER + data_str.encode()

    try:
        # Create UDP socket and send the packet
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(packet, (SERVER_IP, SERVER_PORT))
        status_label.config(text="Packet sent successfully!", foreground="green")
    except Exception as e:
        status_label.config(text=f"Error sending packet: {e}", foreground="red")
    finally:
        sock.close()

# ====== FUNCTION TO SAVE SETTINGS ======
def save_settings():
    settings = {label: entry.get() for label, entry in zip(labels, entry_fields)}
    with open("settings.json", "w") as f:
        json.dump(settings, f)
    status_label.config(text="Settings saved successfully!", foreground="green")

# ====== FUNCTION TO LOAD SETTINGS ======
def load_settings():
    try:
        with open("settings.json", "r") as f:
            settings = json.load(f)
        for label, entry in zip(labels, entry_fields):
            entry.delete(0, tk.END)
            entry.insert(0, settings.get(label, "0"))
        status_label.config(text="Settings loaded successfully!", foreground="green")
    except Exception as e:
        status_label.config(text=f"Error loading settings: {e}", foreground="red")

# ====== GUI SETUP ======
root = tk.Tk()
root.title("UDP Packet Sender")

# Frame to hold the input fields
frame = ttk.Frame(root, padding="10")
frame.grid(row=0, column=0, padx=10, pady=10)

# Field labels for the 10 float values
labels = [
    "WheelBase", "IMUtoANTx", "IMUtoANTy", "IMUtoANTz", 
    "INSx", "INSy", "INSz", "INSanglex", "INSangley", "INSanglez", "KalmanR", "KalmanQ"
]

# Entry fields for the 10 float values
entry_fields = []
for i, label in enumerate(labels):
    # Label for each field
    ttk.Label(frame, text=label).grid(row=i, column=0, sticky="w", pady=2)
    
    # Entry field with default value set to 0
    entry = ttk.Entry(frame)
    entry.insert(0, "0")  # Default value is 0
    entry.grid(row=i, column=1, pady=2)
    entry_fields.append(entry)

# Send button
send_button = ttk.Button(root, text="Send Packet", command=send_packet)
send_button.grid(row=1, column=0, pady=10)

# Save and Load buttons
save_button = ttk.Button(root, text="Save Settings", command=save_settings)
save_button.grid(row=1, column=1, pady=10)

load_button = ttk.Button(root, text="Load Settings", command=load_settings)
load_button.grid(row=1, column=2, pady=10)

# Status label for feedback messages
status_label = tk.Label(root, text="", foreground="black")  # Use 'tk.Label' instead of 'ttk.Label'
status_label.grid(row=2, column=0, pady=5)

# Start the GUI event loop
root.mainloop()
