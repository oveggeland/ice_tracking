import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    # Load the CSV file into a DataFrame
    df = pd.read_csv("/home/oskar/icetrack/output/stats/stats.csv")

    # Convert timestamp to datetime for easier plotting
    # df['ts'] = pd.to_datetime(df['ts'], unit='s')

    # Create subplots with shared x-axis
    fig, (ax1, ax4) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Plot 1: Time series plot for 'count'
    ax1.plot(df['ts'], df['count'], label='Point count', color='b', linestyle='-')
    ax1.set_ylabel('Point count')  # Change label to Point count
    ax1.grid(True)
    ax1.legend().set_visible(False)  # Remove legend for first subplot
    ax1.yaxis.set_ticks_position('right')  # Move y-ticks to the right side

    # # Plot 2: Time series plot for z-values (z_mean, z_var)
    # ax2.plot(df['ts'], df['z_mean'], label='Mean', color='b')

    # # Calculate standard deviation from variance (z_var)
    # std_dev = np.sqrt(df['z_var'])

    # # Fill between z_mean ± 1 standard deviation
    # ax2.fill_between(df['ts'], df['z_mean'] - std_dev, df['z_mean'] + std_dev, color='b', alpha=0.2, label='Variance')
    
    # # Update ylabel to 'Elevation [m]'
    # ax2.set_ylabel('Elevation [m]')
    # ax2.grid(True)
    # ax2.legend()
    # ax2.yaxis.set_ticks_position('right')  # Move y-ticks to the right side

    # # Plot 3: Time series plot for 'z_exp_mean' and 'z_exp_var'
    # ax3.plot(df['ts'], df['z_exp_mean'], label='Exp. Weighted Mean', color='g', linestyle='-')
    
    # # Calculate standard deviation from 'z_exp_var'
    # exp_std_dev = np.sqrt(df['z_exp_var'])

    # # Fill between z_exp_mean ± 1 standard deviation
    # ax3.fill_between(df['ts'], df['z_exp_mean'] - exp_std_dev, df['z_exp_mean'] + exp_std_dev, color='g', alpha=0.2, label='Exp. Weighted Variance')
    
    # # Update ylabel to 'EW Elevation [m]'
    # ax3.set_ylabel('EW Elevation [m]')
    # ax3.grid(True)
    # ax3.legend()
    # ax3.yaxis.set_ticks_position('right')  # Move y-ticks to the right side

    # Plot 4: Regular variance vs. exponential variance
    #ax4.plot(df['ts'], df['z_exp_mean'], label='Exp. Weighted Mean', color='r', linestyle='--')
    ax4.plot(df['ts'], df['z_exp_var'], label='Exp. Weighted Variance', color='g', linestyle='-')

    # Update ylabel to 'Variance'
    ax4.set_ylabel('Elevation [m]')
    ax4.set_xlabel('Time')  # Add xlabel to bottom plot
    ax4.grid(True)
    ax4.legend()
    ax4.yaxis.set_ticks_position('right')  # Move y-ticks to the right side

    # Adjust layout for better spacing
    plt.tight_layout()

    # Save the plot to a PDF file
    plt.savefig("/home/oskar/icetrack/output/stats/stats.pdf")

    # Show the plots
    plt.show()
