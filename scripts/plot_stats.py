import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    # Load the CSV file into a DataFrame
    df = pd.read_csv("/home/oskar/icetrack/output/stats/stats.csv")

    # Convert timestamp to datetime for easier plotting
    #df['ts'] = pd.to_datetime(df['ts'], unit='s')

    # Plot 1: Time series plot for 'count'
    plt.figure(figsize=(12, 4))

    plt.subplot(2, 1, 1)  # First subplot
    plt.plot(df['ts'], df['count'], label='Point count', color='b', marker='o', linestyle='-')
    plt.ylabel('Point count')  # Change label to Point count
    plt.grid(True)
    plt.xticks(plt.xticks()[0], [])  # Remove x-tick labels
    plt.xlabel('')  # Remove xlabel
    plt.legend().set_visible(False)  # Remove legend for first subplot
    plt.gca().yaxis.set_ticks_position('right')  # Move y-ticks to the right side

    # Plot 2: Time series plot for z-values (z_min, z_max, z_mean)
    plt.subplot(2, 1, 2)  # Second subplot

    # Plot z_mean
    plt.plot(df['ts'], df['z_mean'], label='Mean', color='b')

    # Calculate standard deviation from variance (z_var)
    std_dev = np.sqrt(df['z_var'])

    # Fill between z_mean ± 1 standard deviation
    plt.fill_between(df['ts'], df['z_mean'] - std_dev, df['z_mean'] + std_dev, color='b', alpha=0.2, label='Variance')
    
    plt.plot(df['ts'], df['z_min'], linestyle='dashed', color='orange', label='Min')
    plt.plot(df['ts'], df['z_max'], linestyle='dashed', color='purple', label='Max')
    

    # Update ylabel to 'Elevation [m]'
    plt.ylabel('Elevation [m]')
    plt.xlabel('Time')  # Add xlabel to bottom plot
    plt.grid(True)
    plt.legend()
    plt.gca().yaxis.set_ticks_position('right')  # Move y-ticks to the right side

    plt.tight_layout()

    plt.savefig("/home/oskar/icetrack/output/stats/stats.pdf")
    # Show the plots
    plt.show()
