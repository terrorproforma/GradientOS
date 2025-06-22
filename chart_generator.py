import pandas as pd
import matplotlib.pyplot as plt
import os

# --- Configuration ---
LOG_FILE = "performance_log.csv"
CHART_OUTPUT_FILE = "performance_charts.png"

def generate_charts():
    """
    Reads the performance log CSV, calculates statistics, and generates charts
    to visualize the performance data.
    """
    # 1. Check if the log file exists
    if not os.path.exists(LOG_FILE):
        print(f"ERROR: Log file not found at '{LOG_FILE}'.")
        print("Please run the 'performance_tester.py' script first to generate the log.")
        return

    print(f"Reading performance data from '{LOG_FILE}'...")
    df = pd.read_csv(LOG_FILE)

    # --- 2. Calculate and Print Statistics ---
    print("\n--- Performance Statistics (in milliseconds) ---")
    
    # We are interested in the timing columns
    timing_columns = ['read_ms', 'ik_ms', 'write_ms', 'total_loop_ms']
    
    # Calculate statistics and add a frequency column
    stats = df[timing_columns].describe(percentiles=[.5, .9, .99])
    stats.loc['frequency_hz'] = 1000 / stats.loc['mean']
    
    # Format for better readability
    pd.set_option('display.float_format', '{:.3f}'.format)
    print(stats)
    print("-------------------------------------------------")
    
    # --- 3. Calculate and Print Frequency Target Analysis ---
    print("\n--- Frequency Target Analysis ---")
    total_cycles = len(df)
    
    # Slower than 100Hz (loop time > 10ms)
    slow_cycles = df[df['total_loop_ms'] > 10]
    num_slow = len(slow_cycles)
    percent_slow = (num_slow / total_cycles) * 100
    print(f"Cycles slower than 100Hz (> 10ms): {num_slow}/{total_cycles} ({percent_slow:.2f}%)")

    # Faster than 200Hz (loop time < 5ms)
    fast_cycles = df[df['total_loop_ms'] < 5]
    num_fast = len(fast_cycles)
    percent_fast = (num_fast / total_cycles) * 100
    print(f"Cycles faster than 200Hz (< 5ms): {num_fast}/{total_cycles} ({percent_fast:.2f}%)")
    print("---------------------------------")
    
    
    # --- 4. Generate and Save Plots ---
    print(f"\nGenerating charts and saving to '{CHART_OUTPUT_FILE}'...")
    
    # Create a figure with subplots
    # 2 rows for histograms, 1 row for time series
    fig, axes = plt.subplots(3, 2, figsize=(15, 18))
    fig.suptitle('Performance Analysis', fontsize=20, y=0.95)

    # a) Histograms for distribution analysis
    df['total_loop_ms'].hist(bins=50, ax=axes[0, 0])
    axes[0, 0].set_title('Distribution of Total Loop Time (ms)')
    axes[0, 0].set_xlabel('Time (ms)')
    axes[0, 0].set_ylabel('Frequency')

    df['read_ms'].hist(bins=50, ax=axes[0, 1])
    axes[0, 1].set_title('Distribution of Serial Read Time (ms)')
    axes[0, 1].set_xlabel('Time (ms)')

    df['write_ms'].hist(bins=50, ax=axes[1, 0])
    axes[1, 0].set_title('Distribution of Serial Write Time (ms)')
    axes[1, 0].set_xlabel('Time (ms)')
    axes[1, 0].set_ylabel('Frequency')

    df['ik_ms'].hist(bins=50, ax=axes[1, 1])
    axes[1, 1].set_title('Distribution of IK Solve Time (ms)')
    axes[1, 1].set_xlabel('Time (ms)')

    # b) Time series plot to see performance over time
    df.plot(x='cycle', y='total_loop_ms', ax=axes[2, 0], grid=True)
    axes[2, 0].set_title('Total Loop Time vs. Cycle')
    axes[2, 0].set_xlabel('Cycle Number')
    axes[2, 0].set_ylabel('Time (ms)')
    
    # Plot all components on the second time series chart
    df.plot(x='cycle', y=['read_ms', 'write_ms', 'ik_ms'], ax=axes[2, 1], grid=True,
            title='Component Times vs. Cycle', xlabel='Cycle Number', ylabel='Time (ms)')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Save the figure
    plt.savefig(CHART_OUTPUT_FILE)
    
    print("Charts generated successfully.")

if __name__ == '__main__':
    generate_charts() 