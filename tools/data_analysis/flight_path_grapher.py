import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def cumulative_trapz(y, x):
    """
    Compute the cumulative trapezoidal integration of y with respect to x.
    Returns an array where the i-th element is the integral from x[0] to x[i].
    """
    out = np.zeros_like(y)
    for i in range(1, len(y)):
        dt = x[i] - x[i-1]
        out[i] = out[i-1] + 0.5 * (y[i] + y[i-1]) * dt
    return out

def main(file_path):
    # Read the CSV file with proper encoding and delimiter
    df = pd.read_csv(file_path, delimiter=',', encoding='utf-8-sig')

    # Convert pitch angle from degrees to radians
    pitch_rad = np.deg2rad(df['pitch'].values)

    # Decompose acceleration assuming accel_x is the forward acceleration.
    # Horizontal component: a_horizontal = accel_x * cos(pitch)
    # Vertical component:   a_vertical   = accel_x * sin(pitch)
    horizontal_acc = df['accel_x'].values * np.cos(pitch_rad)
    vertical_acc = df['accel_x'].values * np.sin(pitch_rad)

    # Create a time vector (assume timestamp is in seconds and start at 0)
    time = df['timestamp'].values/ 1000.0  # Convert milliseconds to seconds
    time = time - time[0]

    # Integrate to get velocities and then displacements
    horizontal_velocity = cumulative_trapz(horizontal_acc, time)
    horizontal_disp = cumulative_trapz(horizontal_velocity, time)
    vertical_velocity = cumulative_trapz(vertical_acc, time)
    vertical_disp = cumulative_trapz(vertical_velocity, time)

    # Plot the estimated flight path (Horizontal displacement vs. Altitude displacement)
    plt.figure(figsize=(10, 5))
    plt.plot(horizontal_disp, vertical_disp, marker='o')
    plt.xlabel('Horizontal Displacement (m)')
    plt.ylabel('Altitude Displacement (m)')
    plt.title('Estimated Flight Path (Disregarding Yaw)')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot estimated flight path from glider flight data.')
    parser.add_argument('file', help='Path to the flight data CSV file')
    args = parser.parse_args()
    main(args.file)
