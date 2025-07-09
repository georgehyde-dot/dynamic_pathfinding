import csv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from collections import defaultdict
import seaborn as sns
import argparse
import sys
import os
import warnings

# Suppress warnings for cleaner output
warnings.filterwarnings('ignore')

def validate_csv_columns(df):
    """Validate that required columns exist in the CSV file"""
    required_columns = [
        'algorithm', 'grid_size', 'num_walls', 'num_obstacles', 'success',
        'total_moves', 'optimal_path_length', 'route_efficiency',
        'execution_time_ms', 'average_find_path_time_ns', 'total_pathfinding_calls'
    ]

    missing_columns = [col for col in required_columns if col not in df.columns]
    if missing_columns:
        raise ValueError(f"Missing required columns: {missing_columns}")

    # Check for optional columns
    optional_columns = ['a_star_calls', 'd_star_calls']
    for col in optional_columns:
        if col not in df.columns:
            df[col] = 0

    return df

def calculate_metrics(df):
    """Calculate derived metrics for analysis"""
    df = df.copy()

    # Filter to only include a_star and d_star_lite algorithms
    df = df[df['algorithm'].isin(['a_star', 'd_star_lite'])]

    # Calculate density metrics
    df['grid_area'] = df['grid_size'] ** 2
    df['obstacle_density'] = df['num_obstacles'] / df['grid_area']
    df['wall_density'] = df['num_walls'] / df['grid_area']
    df['total_density'] = (df['num_obstacles'] + df['num_walls']) / df['grid_area']
    df['combined_difficulty'] = df['num_obstacles'] + df['num_walls']

    # Create difficulty categories
    df['difficulty_category'] = pd.cut(
        df['total_density'],
        bins=[0, 0.1, 0.3, 0.5, 1.0],
        labels=['Low', 'Medium', 'High', 'Extreme'],
        include_lowest=True
    )

    # Convert nanoseconds to milliseconds for better readability
    df['find_path_time_ms'] = df['average_find_path_time_ns'] / 1000000

    return df

def safe_plot_line(ax, x_data, y_data, algorithm, marker='o', **kwargs):
    """Safely plot line data with error handling"""
    if len(x_data) == 0 or len(y_data) == 0:
        return

    # Filter out NaN values
    valid_mask = ~(np.isnan(x_data) | np.isnan(y_data))
    if not valid_mask.any():
        return

    x_clean = x_data[valid_mask]
    y_clean = y_data[valid_mask]

    ax.plot(x_clean, y_clean, marker=marker, label=algorithm, linewidth=2, **kwargs)

def create_density_bins(data, column, num_bins=20):
    """Create density bins for better visualization"""
    if data.empty or column not in data.columns:
        return data, []

    min_val = data[column].min()
    max_val = data[column].max()

    if min_val == max_val:
        return data, [min_val]

    bins = np.linspace(min_val, max_val, num_bins)
    data_copy = data.copy()
    data_copy['density_bin'] = pd.cut(data_copy[column], bins=bins)

    return data_copy, bins

def plot_metric_vs_density(ax, df, x_column, y_column, title, x_label, y_label, num_bins=20):
    """Plot metric vs density with proper error handling"""
    if df.empty:
        ax.set_title(f"{title}\n(No data available)")
        return

    colors = {'a_star': 'blue', 'd_star_lite': 'red'}
    markers = {'a_star': 'o', 'd_star_lite': 's'}

    for algorithm in ['a_star', 'd_star_lite']:
        if algorithm not in df['algorithm'].values:
            continue

        algo_data = df[df['algorithm'] == algorithm]

        if algo_data.empty:
            continue

        # Create density bins
        algo_data_binned, bins = create_density_bins(algo_data, x_column, num_bins)

        if len(bins) <= 1:
            continue

        # Group by density bins and calculate mean
        density_groups = algo_data_binned.groupby('density_bin')[y_column].mean()

        # Extract bin centers and valid values
        bin_centers = []
        valid_values = []
        for interval, value in density_groups.items():
            if pd.notna(interval) and pd.notna(value):
                bin_centers.append(interval.mid)
                valid_values.append(value)

        if len(bin_centers) > 0:
            ax.plot(bin_centers, valid_values, marker=markers[algorithm],
                   color=colors[algorithm], label=algorithm, linewidth=2)

    ax.set_title(title)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.legend()
    ax.grid(True, alpha=0.3)

def plot_metric_vs_grid_size(ax, df, y_column, title, y_label, marker='s'):
    """Plot metric vs grid size with proper error handling"""
    if df.empty:
        ax.set_title(f"{title}\n(No data available)")
        return

    colors = {'a_star': 'blue', 'd_star_lite': 'red'}
    markers = {'a_star': 'o', 'd_star_lite': 's'}

    for algorithm in ['a_star', 'd_star_lite']:
        if algorithm not in df['algorithm'].values:
            continue

        algo_data = df[df['algorithm'] == algorithm]

        if algo_data.empty:
            continue

        size_groups = algo_data.groupby('grid_size')[y_column].mean()

        if len(size_groups) > 0:
            ax.plot(size_groups.index.values, size_groups.values,
                   marker=markers[algorithm], color=colors[algorithm],
                   label=algorithm, linewidth=2)

    ax.set_title(title)
    ax.set_xlabel('Grid Size')
    ax.set_ylabel(y_label)
    ax.legend()
    ax.grid(True, alpha=0.3)

def create_visualizations(df, successful_df, failed_df):
    """Create all 12 visualizations"""
    # Create figure with subplots - 4x3 layout for 12 plots
    fig = plt.figure(figsize=(20, 24))

    # 1. Find Path Time vs Obstacle Density (successful runs)
    ax1 = plt.subplot(4, 3, 1)
    plot_metric_vs_density(ax1, successful_df, 'obstacle_density', 'find_path_time_ms',
                          'Find Path Time vs Obstacle Density\n(Successful Runs)',
                          'Obstacle Density', 'Find Path Time (ms)')

    # 2. Find Path Time vs Grid Size (successful runs)
    ax2 = plt.subplot(4, 3, 2)
    plot_metric_vs_grid_size(ax2, successful_df, 'find_path_time_ms',
                            'Find Path Time vs Grid Size\n(Successful Runs)',
                            'Find Path Time (ms)')

    # 3. Success Rate vs Obstacle Density
    ax3 = plt.subplot(4, 3, 3)
    plot_metric_vs_density(ax3, df, 'obstacle_density', 'success',
                          'Success Rate vs Obstacle Density\n(All Runs)',
                          'Obstacle Density', 'Success Rate')

    # 4. Success Rate vs Grid Size
    ax4 = plt.subplot(4, 3, 4)
    plot_metric_vs_grid_size(ax4, df, 'success',
                            'Success Rate vs Grid Size\n(All Runs)',
                            'Success Rate')

    # 5. Pathfinding Calls vs Obstacle Density
    ax5 = plt.subplot(4, 3, 5)
    plot_metric_vs_density(ax5, successful_df, 'obstacle_density', 'total_pathfinding_calls',
                          'Pathfinding Calls vs Obstacle Density\n(Successful Runs)',
                          'Obstacle Density', 'Pathfinding Calls')

    # 6. Pathfinding Calls vs Grid Size
    ax6 = plt.subplot(4, 3, 6)
    plot_metric_vs_grid_size(ax6, successful_df, 'total_pathfinding_calls',
                            'Pathfinding Calls vs Grid Size\n(Successful Runs)',
                            'Pathfinding Calls')

    # 7. Execution Time vs Total Density
    ax7 = plt.subplot(4, 3, 7)
    plot_metric_vs_density(ax7, successful_df, 'total_density', 'execution_time_ms',
                          'Execution Time vs Total Density\n(Successful Runs)',
                          'Total Density', 'Execution Time (ms)')

    # 8. Execution Time vs Grid Size
    ax8 = plt.subplot(4, 3, 8)
    plot_metric_vs_grid_size(ax8, successful_df, 'execution_time_ms',
                            'Execution Time vs Grid Size\n(Successful Runs)',
                            'Execution Time (ms)')

    # 9. Find Path Time Distribution by Difficulty (Box plot)
    ax9 = plt.subplot(4, 3, 9)
    if not successful_df.empty and 'difficulty_category' in successful_df.columns:
        try:
            box_data = []
            box_labels = []
            colors = ['blue', 'red']

            for i, algorithm in enumerate(['a_star', 'd_star_lite']):
                if algorithm not in successful_df['algorithm'].values:
                    continue

                algo_data = successful_df[successful_df['algorithm'] == algorithm]
                for category in ['Low', 'Medium', 'High', 'Extreme']:
                    cat_data = algo_data[algo_data['difficulty_category'] == category]
                    if len(cat_data) > 0:
                        box_data.append(cat_data['find_path_time_ms'].values)
                        box_labels.append(f"{algorithm}\n{category}")

            if box_data:
                bp = ax9.boxplot(box_data, labels=box_labels, patch_artist=True)

                # Color the boxes
                for i, patch in enumerate(bp['boxes']):
                    algorithm = box_labels[i].split('\n')[0]
                    color = 'lightblue' if algorithm == 'a_star' else 'lightcoral'
                    patch.set_facecolor(color)

                ax9.set_title('Find Path Time Distribution by Difficulty')
                ax9.set_ylabel('Find Path Time (ms)')
                plt.setp(ax9.get_xticklabels(), rotation=45)
                ax9.grid(axis='y', alpha=0.3)
            else:
                ax9.set_title('Find Path Time Distribution\n(No data available)')
        except Exception as e:
            ax9.set_title(f'Find Path Time Distribution\n(Error: {str(e)})')
    else:
        ax9.set_title('Find Path Time Distribution\n(No data available)')

    # 10. Performance Degradation vs Grid Size
    ax10 = plt.subplot(4, 3, 10)
    if not successful_df.empty:
        colors = {'a_star': 'blue', 'd_star_lite': 'red'}
        markers = {'a_star': 'o', 'd_star_lite': 's'}

        for algorithm in ['a_star', 'd_star_lite']:
            if algorithm not in successful_df['algorithm'].values:
                continue

            algo_data = successful_df[successful_df['algorithm'] == algorithm]
            size_groups = algo_data.groupby('grid_size')['find_path_time_ms'].mean()

            if len(size_groups) > 1:
                base_time = size_groups.iloc[0]
                if base_time > 0:
                    normalized_time = size_groups / base_time
                    ax10.plot(size_groups.index.values, normalized_time.values,
                             marker=markers[algorithm], color=colors[algorithm],
                             label=algorithm, linewidth=2)

        ax10.set_title('Performance Degradation vs Grid Size\n(Normalized to Smallest Grid)')
        ax10.set_xlabel('Grid Size')
        ax10.set_ylabel('Performance Ratio')
        ax10.legend()
        ax10.grid(True, alpha=0.3)
    else:
        ax10.set_title('Performance Degradation\n(No data available)')

    # 11. Find Path Time on Failed Runs vs Grid Size
    ax11 = plt.subplot(4, 3, 11)
    if not failed_df.empty:
        colors = {'a_star': 'blue', 'd_star_lite': 'red'}
        markers = {'a_star': 'o', 'd_star_lite': 's'}

        for algorithm in ['a_star', 'd_star_lite']:
            if algorithm not in failed_df['algorithm'].values:
                continue

            algo_data = failed_df[failed_df['algorithm'] == algorithm]
            size_groups = algo_data.groupby('grid_size')['find_path_time_ms'].mean()

            if len(size_groups) > 0:
                ax11.plot(size_groups.index.values, size_groups.values,
                         marker=markers[algorithm], color=colors[algorithm],
                         label=algorithm, linewidth=2)

        ax11.set_title('Find Path Time on Failed Runs vs Grid Size')
        ax11.set_xlabel('Grid Size')
        ax11.set_ylabel('Find Path Time (ms)')
        ax11.legend()
        ax11.grid(True, alpha=0.3)
    else:
        ax11.set_title('Find Path Time on Failed Runs\n(No failed runs)')

    # 12. Additional Graph: Algorithm Efficiency vs Problem Complexity
    ax12 = plt.subplot(4, 3, 12)
    if not successful_df.empty:
        colors = {'a_star': 'blue', 'd_star_lite': 'red'}

        for algorithm in ['a_star', 'd_star_lite']:
            if algorithm not in successful_df['algorithm'].values:
                continue

            algo_data = successful_df[successful_df['algorithm'] == algorithm]

            # Calculate efficiency ratio: route efficiency / pathfinding calls
            algo_data = algo_data.copy()
            algo_data['efficiency_ratio'] = algo_data['route_efficiency'] / (algo_data['total_pathfinding_calls'] + 1)

            # Plot efficiency ratio vs combined difficulty
            ax12.scatter(algo_data['combined_difficulty'], algo_data['efficiency_ratio'],
                        alpha=0.6, color=colors[algorithm], label=algorithm, s=30)

        ax12.set_title('Algorithm Efficiency vs Problem Complexity\n(Route Efficiency / Pathfinding Calls)')
        ax12.set_xlabel('Combined Difficulty (Walls + Obstacles)')
        ax12.set_ylabel('Efficiency Ratio')
        ax12.legend()
        ax12.grid(True, alpha=0.3)
    else:
        ax12.set_title('Algorithm Efficiency\n(No data available)')

    plt.tight_layout()
    return fig

def print_analysis_results(df, successful_df, failed_df, quiet=False):
    """Print comprehensive analysis results"""
    if quiet:
        return

    print("\n=== ALGORITHM COMPARISON ANALYSIS (A* vs D* Lite) ===")

    # Success Rate Analysis
    print("\n1. Success Rate Analysis:")
    success_stats = df.groupby('algorithm')['success'].agg(['count', 'sum', 'mean'])
    success_stats['rate'] = success_stats['mean'] * 100

    for algorithm in ['a_star', 'd_star_lite']:
        if algorithm in success_stats.index:
            stats = success_stats.loc[algorithm]
            print(f"  {algorithm}: {stats['rate']:.1f}% success ({stats['sum']}/{stats['count']} runs)")

    # Performance metrics for successful runs
    if not successful_df.empty:
        print("\n2. Performance Metrics (Successful Runs Only):")

        perf_stats = successful_df.groupby('algorithm').agg({
            'find_path_time_ms': ['mean', 'std'],
            'execution_time_ms': ['mean', 'std'],
            'total_pathfinding_calls': ['mean', 'std'],
            'route_efficiency': ['mean', 'std']
        }).round(3)

        for algorithm in ['a_star', 'd_star_lite']:
            if algorithm in perf_stats.index:
                print(f"\n  {algorithm}:")
                print(f"    Find Path Time: {perf_stats.loc[algorithm, ('find_path_time_ms', 'mean')]:.3f} ± {perf_stats.loc[algorithm, ('find_path_time_ms', 'std')]:.3f} ms")
                print(f"    Execution Time: {perf_stats.loc[algorithm, ('execution_time_ms', 'mean')]:.3f} ± {perf_stats.loc[algorithm, ('execution_time_ms', 'std')]:.3f} ms")
                print(f"    Pathfinding Calls: {perf_stats.loc[algorithm, ('total_pathfinding_calls', 'mean')]:.1f} ± {perf_stats.loc[algorithm, ('total_pathfinding_calls', 'std')]:.1f}")
                print(f"    Route Efficiency: {perf_stats.loc[algorithm, ('route_efficiency', 'mean')]:.3f} ± {perf_stats.loc[algorithm, ('route_efficiency', 'std')]:.3f}")

    # Summary Statistics
    print(f"\n=== SUMMARY STATISTICS ===")
    print(f"Total records: {len(df)}")
    print(f"Successful runs: {len(successful_df)}")
    print(f"Failed runs: {len(failed_df)}")
    print(f"Overall success rate: {len(successful_df) / len(df) * 100:.1f}%")
    print(f"Grid sizes: {sorted(df['grid_size'].unique())}")
    print(f"Difficulty range: {df['combined_difficulty'].min()}-{df['combined_difficulty'].max()}")

def main():
    """Main function to run the analysis"""
    # Command line argument parsing
    parser = argparse.ArgumentParser(description='Analyze A* vs D* Lite pathfinding performance')
    parser.add_argument('input_csv', help='Input CSV file path')
    parser.add_argument('output_image', help='Output image file path (e.g., analysis.png)')
    parser.add_argument('--quiet', '-q', action='store_true', help='Suppress printed output')

    args = parser.parse_args()

    # Validate input file
    if not os.path.exists(args.input_csv):
        print(f"Error: Input file '{args.input_csv}' does not exist")
        sys.exit(1)

    # Validate output directory
    output_dir = os.path.dirname(args.output_image)
    if output_dir and not os.path.exists(output_dir):
        print(f"Error: Output directory '{output_dir}' does not exist")
        sys.exit(1)

    # Set up plotting style
    plt.style.use('default')
    sns.set_palette("husl")

    try:
        # Read and validate CSV
        df = pd.read_csv(args.input_csv)
        df = validate_csv_columns(df)

        # Calculate derived metrics (includes filtering to a_star and d_star_lite only)
        df = calculate_metrics(df)

        if df.empty:
            print("Error: No data found for a_star or d_star_lite algorithms")
            sys.exit(1)

        if not args.quiet:
            print(f"Loaded {len(df)} records from {args.input_csv}")
            print(f"Algorithms found: {df['algorithm'].unique()}")

        # Filter successful and failed runs
        successful_df = df[df['success'] == True]
        failed_df = df[df['success'] == False]

        if not args.quiet:
            print(f"Successful runs: {len(successful_df)}")
            print(f"Failed runs: {len(failed_df)}")

        # Create visualizations
        fig = create_visualizations(df, successful_df, failed_df)

        # Save figure
        fig.savefig(args.output_image, dpi=300, bbox_inches='tight')

        if not args.quiet:
            print(f"Graph saved to {args.output_image}")

        # Print analysis results
        print_analysis_results(df, successful_df, failed_df, args.quiet)

        if not args.quiet:
            plt.show()

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
