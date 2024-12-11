#! /bin/python3 

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

DEG2RAD = (np.pi / 180)
RAD2DEG = (1 / DEG2RAD)

def plot_navigation(nav_data):
    # Create the figure and subplots (3x2 layout)
    fig, axs = plt.subplots(3, 2, figsize=(12, 10), num="Overview")
    pos_ax, v_ax, rot_ax, acc_bias_ax, gyro_bias_ax, height_ax = axs.flatten()

    # Cartesian position (North-East)
    cmap = pos_ax.scatter(nav_data["y"], nav_data["x"], c=nav_data["ts"], cmap='viridis')
    pos_ax.set_xlabel('East [m]')
    pos_ax.set_ylabel('North [m]')
    pos_ax.set_title('Cartesian position')
    fig.colorbar(cmap, ax=pos_ax, label='Time [s]')

    # Velocity
    v_ax.plot(nav_data["ts"].values, nav_data["vx"].values, label="Velocity X")
    v_ax.plot(nav_data["ts"].values, nav_data["vy"].values, label="Velocity Y")
    v_ax.plot(nav_data["ts"].values, nav_data["vz"].values, label="Velocity Z")
    v_ax.set_xlabel('Time [s]')
    v_ax.set_ylabel('Velocity [m/s]')
    v_ax.legend()

    # Orientation (Roll, Pitch, Yaw)
    rot_ax.plot(nav_data["ts"].values, nav_data["roll"].values, label='Roll')
    rot_ax.plot(nav_data["ts"].values, nav_data["pitch"].values, label='Pitch')
    rot_ax.plot(nav_data["ts"].values, nav_data["yaw"].values, label='Yaw')
    rot_ax.set_xlabel('Time [s]')
    rot_ax.set_ylabel('Angle [rad]')
    rot_ax.legend()

    # Acceleration bias
    acc_bias_ax.plot(nav_data["ts"].values, nav_data["bax"].values, label='Acc Bias X')
    acc_bias_ax.plot(nav_data["ts"].values, nav_data["bay"].values, label='Acc Bias Y')
    acc_bias_ax.plot(nav_data["ts"].values, nav_data["baz"].values, label='Acc Bias Z')
    acc_bias_ax.set_xlabel('Time [s]')
    acc_bias_ax.set_ylabel('Acceleration Bias [m/s²]')
    acc_bias_ax.legend()

    # Gyroscope bias
    gyro_bias_ax.plot(nav_data["ts"].values, nav_data["bgx"].values, label='Gyro Bias X')
    gyro_bias_ax.plot(nav_data["ts"].values, nav_data["bgy"].values, label='Gyro Bias Y')
    gyro_bias_ax.plot(nav_data["ts"].values, nav_data["bgz"].values, label='Gyro Bias Z')
    gyro_bias_ax.set_xlabel('Time [s]')
    gyro_bias_ax.set_ylabel('Gyroscope Bias [rad/s]')
    gyro_bias_ax.legend()

    # Relative height
    height_ax.plot(nav_data["ts"].values, nav_data["z"].values, label="Altitude")
    #height_ax.plot(nav_data["ts"].values, nav_data["bz"].values, label="Altitude bias")
    height_ax.set_xlabel('Time [s]')
    height_ax.set_ylabel('Vertical displacement [m]')

    # Adjust layout to avoid overlapping of labels
    plt.tight_layout(pad=3.0)


    plt.figure()
    plt.plot(nav_data['ts'].values, nav_data[["Lx", "Ly", "Lz"]], label=["Lx", "Ly", "Lz"])
    plt.legend()
    plt.tight_layout()


    
if __name__ == "__main__":
    # Extract navigation estimates
    nav_data = pd.read_csv("/home/oskar/icetrack/output/nav/nav.csv")

    plot_navigation(nav_data)
    plt.savefig("/home/oskar/icetrack/output/nav/nav.png")
    plt.show()
    
    
