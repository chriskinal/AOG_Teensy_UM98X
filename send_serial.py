import serial
import time
import numpy as np

# Function to generate sinusoidal data
def generate_sin_wave(amplitude, frequency, duration, sample_rate=100):
    t = np.linspace(0, duration, int(sample_rate * duration))
    data = amplitude * np.sin(2 * np.pi * frequency * t)
    return data

# Virtual serial port setup
port = '/dev/ttys006'  # Replace with your virtual serial port
baud_rate = 115200
ser = serial.Serial(port, baud_rate, timeout=1)

# Parameters for sinusoidal data
amplitude = 1.0  # Amplitude of the sine wave
frequencies = [0.5, 1.0, 1.5, 2.0]  # Different frequencies for each variable
duration = 10  # Duration in seconds

try:
    print("Simulating serial data...")
    while True:
        # Generate sinusoidal data for each variable
        steerAngleSens = generate_sin_wave(amplitude, frequencies[0], duration)
        insWheelAngle = generate_sin_wave(amplitude, frequencies[1], duration)
        keyaEncoder = generate_sin_wave(amplitude, frequencies[2], duration)
        KalmanWheelAngle = generate_sin_wave(amplitude, frequencies[3], duration)

        # Send data over the serial port
        for i in range(len(steerAngleSens)):
            # Format the data as per your requirements
            data = (
                f"sens:{steerAngleSens[i]:.2f},"
                f"insAngle:{insWheelAngle[i]:.2f},"
                f"keyaAngle:{keyaEncoder[i]:.2f},"
                f"Kalman:{KalmanWheelAngle[i]:.2f}\n"
            )
            ser.write(data.encode('utf-8'))  # Send data over serial
            time.sleep(0.01)  # Adjust the delay as needed

        print("Data sent. Restarting simulation...")

except KeyboardInterrupt:
    print("Simulation stopped.")

finally:
    ser.close()  # Close the serial port