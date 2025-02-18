import socket
import time
import random
import math
from datetime import datetime
import argparse

class TeensySimulator:
    def __init__(self, target_ip='127.0.0.1'):
        self.target_ip = target_ip
        self.plot_port = 6968
        self.debug_port = 6969
        
        # Create UDP sockets
        self.plot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.debug_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Initialize simulated values
        self.base_time = time.time()
        self.steerAngleSens = 0.0
        self.insWheelAngle = 0.0
        self.keyaEncoder = 0.0
        self.KalmanWheelAngle = 0.0
        self.angleVariance = 0.0
        
    def send_debug_message(self, message):
        message = f"[SIM]{datetime.now().strftime('%H:%M:%S')} {message}"
        self.debug_sock.sendto(message.encode(), (self.target_ip, self.debug_port))
        
    def generate_sensor_data(self):
        # Simulate realistic steering angles with some noise and relationships
        t = (time.time() - self.base_time) * 0.3  # Slow down time factor
        
        self.steerAngleSens = 30 * math.sin(t)  # Primary steering signal
        self.insWheelAngle = self.steerAngleSens + random.gauss(0, 0.5)  # Noisy version
        self.keyaEncoder = (t * 10) % 100  # Sawtooth pattern
        self.KalmanWheelAngle = 0.9 * self.KalmanWheelAngle + 0.1 * self.steerAngleSens  # Filtered
        self.angleVariance = abs(random.gauss(0, 0.1))  # Always positive
        
    def send_plot_data(self):
        timestamp = int((time.time() - self.base_time) * 1000)
        data_str = (f"{timestamp},"
                    f"{self.steerAngleSens:.3f},"
                    f"{self.insWheelAngle:.3f},"
                    f"{self.keyaEncoder:.3f},"
                    f"{self.KalmanWheelAngle:.3f},"
                    f"{self.angleVariance:.5f}")
        
        self.plot_sock.sendto(data_str.encode(), (self.target_ip, self.plot_port))
        
    def run(self):
        try:
            print(f"Starting Teensy simulator, sending data to {self.target_ip}")
            while True:
                # Update simulated values
                self.generate_sensor_data()
                
                # Send plot data every 100ms
                self.send_plot_data()
                
                # Send debug messages randomly
                if random.random() < 0.1:  # 10% chance each cycle
                    self.send_debug_message("Normal operation - all systems OK")
                    
                if random.random() < 0.02:  # 2% chance each cycle
                    self.send_debug_message("WARNING: High variance detected!")
                    
                time.sleep(0.1)  # Main loop at 10Hz
                
        except KeyboardInterrupt:
            print("\nSimulation stopped")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Teensy 4.1 Simulator')
    parser.add_argument('--ip', default='127.0.0.1', help='Target IP address')
    args = parser.parse_args()

    simulator = TeensySimulator(target_ip=args.ip)
    simulator.run()