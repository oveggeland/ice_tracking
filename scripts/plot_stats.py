import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    # Load the CSV file into a DataFrame
    df = pd.read_csv("/home/oskar/icetrack/output/stats/stats.csv")

    # Convert timestamp to datetime for easier plotting
    #df['ts'] = pd.to_datetime(df['ts'], unit='s')

    # Create subplots with shared x-axis
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot 1: Time series plot for 'count'
    ax1.plot(df['ts'], df['count'], label='Point count', color='b', marker='o', linestyle='-')
    ax1.set_ylabel('Point count')  # Change label to Point count
    ax1.grid(True)
    ax1.legend().set_visible(False)  # Remove legend for first subplot
    ax1.yaxis.set_ticks_position('right')  # Move y-ticks to the right side
    #ax1.set_xticklabels([])  # Remove x-tick labels for the first subplot

    # Plot 2: Time series plot for z-values (z_min, z_max, z_mean)
    ax2.plot(df['ts'], df['z_mean'], label='Mean', color='b')

    # Calculate standard deviation from variance (z_var)
    std_dev = np.sqrt(df['z_var'])

    # Fill between z_mean ± 1 standard deviation
    ax2.fill_between(df['ts'], df['z_mean'] - std_dev, df['z_mean'] + std_dev, color='b', alpha=0.2, label='Variance')
    
    # plt.plot(df['ts'], df['z_min'], linestyle='dashed', color='orange', label='Min')
    # plt.plot(df['ts'], df['z_max'], linestyle='dashed', color='purple', label='Max')
    
    # Update ylabel to 'Elevation [m]'
    ax2.set_ylabel('Elevation [m]')
    ax2.set_xlabel('Time')  # Add xlabel to bottom plot
    ax2.grid(True)
    ax2.legend()
    ax2.yaxis.set_ticks_position('right')  # Move y-ticks to the right side
    

    # Adjust layout for better spacing
    plt.tight_layout()

    # Save the plot to a PDF file
    plt.savefig("/home/oskar/icetrack/output/stats/stats.pdf")

    # Show the plots
    plt.show()