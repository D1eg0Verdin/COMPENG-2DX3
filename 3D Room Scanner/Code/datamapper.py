"""
3D Room Scanner Data Logger and Visualizer
------------------------------------------
This program reads distance and angle measurements from a microcontroller
via serial communication. The data is converted into Cartesian coordinates
(x, y, z) to construct a 3D point cloud representation of a scanned space. 

Features:
- Collects vertical sweeps of points (defined by POINTS_PER_SCAN).
- Advances manually in the x-direction after each sweep.
- Saves scan data to a CSV file.
- Produces a 3D plot of the reconstructed environment.

Author: Diego Verdin Ramirez
Course: COMPENG 2DX3 – Microprocessor Systems
"""

# ----------------------------
# Import Required Libraries
# ----------------------------
import serial           # For serial communication with microcontroller
import pandas as pd     # For data storage and manipulation
import numpy as np      # For numerical operations
import matplotlib.pyplot as plt  # For 3D plotting
import math             # For trigonometric conversions

# ----------------------------
# User Configuration
# ----------------------------
PORT = 'COM4'           # Serial port connected to the microcontroller
BAUD = 115200           # Baud rate for communication
POINTS_PER_SCAN = 32    # Number of distance-angle measurements per vertical sweep
X_STEP = 100            # Distance moved forward (mm) after each sweep
SAVE_TO_CSV = True      # Enable saving data to CSV file

# ----------------------------
# Serial Setup
# ----------------------------
ser = serial.Serial(PORT, BAUD, timeout=10)

# ----------------------------
# Data Structures
# ----------------------------
data = []               # Holds collected (x, y, z) points
x_position = 0          # Tracks manual displacement along the x-axis
point_counter = 0       # Counts points within the current scan

print("Listening for ToF data... (press Ctrl+C to stop)")

# ----------------------------
# Data Acquisition Loop
# ----------------------------
try:
    while True:
        # Read one line from the serial port
        line = ser.readline().decode('utf-8').strip()
        print(f"Raw line: '{line}'")  # Debug: show raw input

        if not line:
            continue  # Skip if no data received

        try:
            # Expected format: "distance, angle"
            distance_str, angle_str = line.split(',')
            distance = float(distance_str.strip())      # Distance in mm
            angle_deg = float(angle_str.strip())        # Angle in degrees

            # Convert polar coordinates to Cartesian (y, z)
            angle_rad = math.radians(angle_deg)
            y = distance * math.cos(angle_rad)
            z = distance * math.sin(angle_rad)
            x = x_position  # x is based on manual displacement per scan

            # Append point to dataset
            data.append([x, y, z])

            # Increment point counter for current sweep
            point_counter += 1
            if point_counter == POINTS_PER_SCAN:
                # Finished one vertical sweep
                point_counter = 0
                x_position += X_STEP  # Move forward in x-axis
                print(f"Advanced to x = {x_position} mm")
                input("Press Enter to continue to next scan...")

        except ValueError:
            # Handles improperly formatted serial data
            print(f"Invalid line skipped: '{line}'")

# Graceful exit on keyboard interrupt
except KeyboardInterrupt:
    print("Stopped by user.")

# ----------------------------
# Data Processing
# ----------------------------
# Store collected data in a DataFrame
df = pd.DataFrame(data, columns=["x", "y", "z"])

# Optionally save to CSV for later use
if SAVE_TO_CSV:
    df.to_csv("scan_data.csv", index=False)
    print("Data saved to 'scan_data.csv'")

# ----------------------------
# Visualization
# ----------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot vertical sweeps (lines along z-axis for each x position)
for i in range(0, len(df), POINTS_PER_SCAN):
    batch = df.iloc[i:i+POINTS_PER_SCAN]
    if len(batch) == POINTS_PER_SCAN:
        ax.plot(batch["x"], batch["y"], batch["z"], linewidth=1.2)

# Plot horizontal connections (linking sweeps together)
num_scans = len(df) // POINTS_PER_SCAN
for i in range(num_scans - 1):
    scan1 = df.iloc[i * POINTS_PER_SCAN:(i + 1) * POINTS_PER_SCAN]
    scan2 = df.iloc[(i + 1) * POINTS_PER_SCAN:(i + 2) * POINTS_PER_SCAN]
    for j in range(POINTS_PER_SCAN):
        x = [scan1.iloc[j]["x"], scan2.iloc[j]["x"]]
        y = [scan1.iloc[j]["y"], scan2.iloc[j]["y"]]
        z = [scan1.iloc[j]["z"], scan2.iloc[j]["z"]]
        ax.plot(x, y, z, color='gray', linewidth=0.6)

# Label axes and add title
ax.set_xlabel("X (mm) — Manual Displacement")
ax.set_ylabel("Y (mm) — Forward")
ax.set_zlabel("Z (mm) — Vertical Scan")
ax.set_title("3D Hallway Map (From Logged Serial Data)")

# Adjust layout and display plot
plt.tight_layout()
plt.show()