# AgOpenGPS WAS-less Firmware for Teensy 4.1, UM981/UM982, and KeyaMotor

## Overview
This firmware integrates the **UM981** for roll and pitch-corrected positioning with true orientation using a **single antenna**. It is designed for use with a **Teensy 4.1**, **UM981 or UM982**, and **KeyaMotor**. 
The **first** well-working **WAS-less** algorithm and UM981 GPS receiver.

## Features
- **Single-antenna positioning** with roll and pitch correction.
- **GUI-based configuration** using `GUI_settings.py`.
- **Real-time data visualization** via `GUI_plots.py` (UDP-based serial output, sensor data plotting, and configuration updates).
- **Kalman filter-based wheel angle estimation**, leveraging:
  - Keya Motor encoder.
  - True vehicle heading from UM981 corrected position or UM982 dual-antenna setup.
  - Sensor fusion, where the encoder dominates during rapid changes, while true heading compensates for drift.
- Designed for a **custom PCB**.

## Configuration & Usage
### AgOpenGPS Settings Adjustments
- **Pressure Sensor Active** → Used for debugging via the guide activation button (controls serial output data).
- **Invert WAS** → If enabled, a green angle sensor is used; otherwise, the wheel angle is estimated.
- **isDanfoss Button (Green When Sending Settings from AOG)** → Saves settings for Keya; calibration is crucial:
  - **Offset Calibration**: Represents the "empty run" of the steering wheel before affecting the wheels.
  - **Sensor Counts**: Must be precise to ensure accurate wheel angle compensation.
  - **Ackerman Fix**: Implemented but rarely needed (default: `100`).
- **Cytron/IBT2 Toggle for Multi-Tractor Support**:
  - Some tractor-specific measurements (e.g., antenna & UM981 position) are stored in the **Teensy’s memory**.
  - These settings are sent by `GUI_settings.py` and differ per tractor.
  - **Cytron/IBT2 setting allows switching between tractors**:
    - When measurements are sent, they are stored in the active profile.
    - On startup, the active profile configures UM981 accordingly.

### Steering Wheel Sensor Improvement
- Enhanced detection of **manual steering input** by using a **running average** of the ratio between current and encoder speed.
- Ensures a **highly sensitive threshold** for detecting manual steering.

## Installation & Setup
1. Configure **UM981/UM982** as per **ArduSimple** guidelines.
2. Use `GUI_settings.py` to send settings to the **Teensy**.
3. Use `GUI_plots.py` for real-time data visualization and debugging.
4. Ensure proper calibration of the **wheel angle estimation algorithm**.
5. Use the **Cytron/IBT2 toggle** if switching between tractors.

---
This firmware enhances precision and usability for AgOpenGPS users by integrating advanced sensor fusion and real-time configurability. 🚜⚡
