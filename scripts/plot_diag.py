import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


MSG_TYPES = [
    "IMU",
    "GNSS",
    "LiDAR"
]

DIAG_DIR = "/home/oskar/icetrack/output/diag/"


def reset_cumsum(arr):
    sum = 0
    result = np.zeros_like(arr)
    for i in range(len(arr)):
        sum += arr[i]
        if sum < 0:
            sum = 0  # Reset to zero if cumulative sum is negative
        result[i] = sum
    return result


if __name__ == "__main__":
    # Read the CSV file into a pandas DataFrame
    filename = DIAG_DIR + "diag.csv"  # Replace with your file path if necessary
    data = pd.read_csv(filename)

    # Calculate rolling differences for t_stamp and t0_wall
    window_size = 1
    data['rolling_delta_t_stamp'] = data['t_stamp'].diff(window_size)
    data['rolling_delta_t0_wall'] = data['t0_wall'].diff(window_size)

    # Calculate the Real Time Factor (RTF) based on rolling differences
    data['real_time_factor'] = data['rolling_delta_t_stamp'] / data['rolling_delta_t0_wall']

    # Calculate per-frame processing time (t1_wall - t0_wall)
    data['processing_time'] = data['t1_wall'] - data['t0_wall']

    # Plot 1: Real Time Factor as a time series
    plt.figure(figsize=(10, 6))
    plt.plot(data['t_stamp'], data['real_time_factor'], label='RTF', color='blue')
    plt.xlabel('Timestamp')
    plt.ylabel('Real Time Factor (RTF)')
    plt.title('Real Time Factor over Time')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.savefig(DIAG_DIR + "rtf.pdf")

    # # Plot 2: Per-frame processing time, grouped by msg_type
    # plt.figure(figsize=(10, 6))
    # for msg_type, group in data.groupby('msg_type'):
    #     plt.scatter(group['t_stamp'], group['processing_time'], label=MSG_TYPES[msg_type], s=10)
    # plt.xlabel('Timestamp')
    # plt.ylabel('Processing Time')
    # plt.yscale('log')
    # plt.title('Per-frame Processing Time by Message Type')
    # plt.grid()
    # plt.legend()
    # plt.tight_layout()
    # plt.savefig(DIAG_DIR + "processing_time.pdf")

    # # Plot 3: RAM as a time series
    # plt.figure(figsize=(10, 6))
    # plt.plot(data['t_stamp'], data['mem0'], label='RAM0', color='green')
    # plt.plot(data['t_stamp'], data['mem1'], label='RAM1', color='purple')
    # plt.xlabel('Timestamp')
    # plt.ylabel('Memory Usage (MB)')
    # plt.title('RAM Usage over Time')
    # plt.grid()
    # plt.legend()
    # plt.tight_layout()
    # plt.savefig(DIAG_DIR + "ram_usage.pdf")
    
    # Plot 4: Latency
    data['delta_t_stamp'] = data['t_stamp'].diff()
    data['delta_t0_wall'] = data['t0_wall'].diff()
    
    data['delta_latency'] = data['delta_t0_wall'] - data['delta_t_stamp']
    data = data.dropna()

    data['cum_latency'] = reset_cumsum(data['delta_latency'].values)
    
    plt.figure(figsize=(10, 6))
    plt.plot(data['t_stamp'], data['cum_latency'])
    plt.xlabel('Timestamp')
    plt.ylabel('Latency (s)')
    plt.title('Output latency')
    plt.grid()
    plt.tight_layout()
    plt.savefig(DIAG_DIR + "latency.pdf")
    
    plt.show()