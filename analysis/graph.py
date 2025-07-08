import csv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from collections import defaultdict
import seaborn as sns

# Set up the plotting style
plt.style.use('default')
sns.set_palette("husl")

# Read the CSV file
df = pd.read_csv('performance_test2.csv')

# Filter out hybrid algorithm
df = df[df['algorithm'] != 'hybrid']

# Filter out failed simulations for some analyses
successful_df = df[df['success'] == True]

# Create figure with fewer subplots (removing bar graphs)
fig = plt.figure(figsize=(20, 10))

# 1. Find Path Time vs Number of Obstacles (A* vs D* Lite)
plt.subplot(2, 3, 1)
if not successful_df.empty:
    line_data = {}
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        obstacle_groups = algo_data.groupby('num_obstacles')['average_find_path_time_ns'].mean() / 1000000
        plt.plot(obstacle_groups.index, obstacle_groups.values, marker='o', label=algorithm, linewidth=2)
        line_data[algorithm] = obstacle_groups
    
    plt.title('Find Path Time vs Number of Obstacles')
    plt.xlabel('Number of Obstacles')
    plt.ylabel('Average Find Path Time (ms)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Calculate overall difference between algorithms
    if len(line_data) == 2:
        algos = list(line_data.keys())
        # Find common obstacle values
        common_obstacles = line_data[algos[0]].index.intersection(line_data[algos[1]].index)
        if len(common_obstacles) > 0:
            values1 = line_data[algos[0]][common_obstacles].values
            values2 = line_data[algos[1]][common_obstacles].values
            diff = np.mean(np.abs(values1 - values2))
            print(f"Find Path Time - Average difference between {algos[0]} and {algos[1]}: {diff:.3f} ms")

# 2. Pathfinding Calls vs Number of Obstacles
plt.subplot(2, 3, 2)
if not successful_df.empty:
    line_data = {}
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        obstacle_groups = algo_data.groupby('num_obstacles')['total_pathfinding_calls'].mean()
        plt.plot(obstacle_groups.index, obstacle_groups.values, marker='s', label=algorithm, linewidth=2)
        line_data[algorithm] = obstacle_groups
    
    plt.title('Pathfinding Calls vs Number of Obstacles')
    plt.xlabel('Number of Obstacles')
    plt.ylabel('Average Pathfinding Calls')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Calculate overall difference between algorithms
    if len(line_data) == 2:
        algos = list(line_data.keys())
        # Find common obstacle values
        common_obstacles = line_data[algos[0]].index.intersection(line_data[algos[1]].index)
        if len(common_obstacles) > 0:
            values1 = line_data[algos[0]][common_obstacles].values
            values2 = line_data[algos[1]][common_obstacles].values
            diff = np.mean(np.abs(values1 - values2))
            print(f"Pathfinding Calls - Average difference between {algos[0]} and {algos[1]}: {diff:.2f} calls")

# 3. Execution Time vs Number of Obstacles
plt.subplot(2, 3, 3)
if not successful_df.empty:
    line_data = {}
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        obstacle_groups = algo_data.groupby('num_obstacles')['execution_time_ms'].mean()
        plt.plot(obstacle_groups.index, obstacle_groups.values, marker='^', label=algorithm, linewidth=2)
        line_data[algorithm] = obstacle_groups
    
    plt.title('Execution Time vs Number of Obstacles')
    plt.xlabel('Number of Obstacles')
    plt.ylabel('Average Execution Time (ms)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Calculate overall difference between algorithms
    if len(line_data) == 2:
        algos = list(line_data.keys())
        # Find common obstacle values
        common_obstacles = line_data[algos[0]].index.intersection(line_data[algos[1]].index)
        if len(common_obstacles) > 0:
            values1 = line_data[algos[0]][common_obstacles].values
            values2 = line_data[algos[1]][common_obstacles].values
            diff = np.mean(np.abs(values1 - values2))
            print(f"Execution Time - Average difference between {algos[0]} and {algos[1]}: {diff:.2f} ms")

# 4. Total Moves vs Number of Obstacles
plt.subplot(2, 3, 4)
if not successful_df.empty:
    line_data = {}
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        obstacle_groups = algo_data.groupby('num_obstacles')['total_moves'].mean()
        plt.plot(obstacle_groups.index, obstacle_groups.values, marker='d', label=algorithm, linewidth=2)
        line_data[algorithm] = obstacle_groups
    
    plt.title('Total Moves vs Number of Obstacles')
    plt.xlabel('Number of Obstacles')
    plt.ylabel('Average Total Moves')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Calculate overall difference between algorithms
    if len(line_data) == 2:
        algos = list(line_data.keys())
        # Find common obstacle values
        common_obstacles = line_data[algos[0]].index.intersection(line_data[algos[1]].index)
        if len(common_obstacles) > 0:
            values1 = line_data[algos[0]][common_obstacles].values
            values2 = line_data[algos[1]][common_obstacles].values
            diff = np.mean(np.abs(values1 - values2))
            print(f"Total Moves - Average difference between {algos[0]} and {algos[1]}: {diff:.2f} moves")

# 5. Scatter plot: Find Path Time vs Pathfinding Calls
plt.subplot(2, 3, 5)
if not successful_df.empty:
    scatter_data = {}
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        plt.scatter(algo_data['total_pathfinding_calls'], 
                   algo_data['average_find_path_time_ns'] / 1000000,
                   label=algorithm, alpha=0.6, s=50)
        scatter_data[algorithm] = {
            'x': algo_data['total_pathfinding_calls'].values,
            'y': algo_data['average_find_path_time_ns'].values / 1000000
        }
    
    plt.title('Find Path Time vs Pathfinding Calls')
    plt.xlabel('Total Pathfinding Calls')
    plt.ylabel('Average Find Path Time (ms)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Calculate average distance between algorithms in scatter plot
    if len(scatter_data) == 2:
        algos = list(scatter_data.keys())
        algo1, algo2 = algos[0], algos[1]
        
        # Calculate euclidean distance between corresponding points
        min_len = min(len(scatter_data[algo1]['x']), len(scatter_data[algo2]['x']))
        if min_len > 0:
            x1, y1 = scatter_data[algo1]['x'][:min_len], scatter_data[algo1]['y'][:min_len]
            x2, y2 = scatter_data[algo2]['x'][:min_len], scatter_data[algo2]['y'][:min_len]
            distances = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
            avg_distance = np.mean(distances)
            print(f"Scatter Plot - Average distance between {algo1} and {algo2}: {avg_distance:.3f}")

# 6. Success Rate vs Number of Obstacles
plt.subplot(2, 3, 6)
line_data = {}
for algorithm in df['algorithm'].unique():
    algo_data = df[df['algorithm'] == algorithm]
    success_by_obstacles = algo_data.groupby('num_obstacles')['success'].mean() * 100
    plt.plot(success_by_obstacles.index, success_by_obstacles.values, 
             marker='o', label=algorithm, linewidth=2)
    line_data[algorithm] = success_by_obstacles

plt.title('Success Rate vs Number of Obstacles')
plt.xlabel('Number of Obstacles')
plt.ylabel('Success Rate (%)')
plt.legend()
plt.grid(True, alpha=0.3)

# Calculate overall difference between algorithms
if len(line_data) == 2:
    algos = list(line_data.keys())
    # Find common obstacle values
    common_obstacles = line_data[algos[0]].index.intersection(line_data[algos[1]].index)
    if len(common_obstacles) > 0:
        values1 = line_data[algos[0]][common_obstacles].values
        values2 = line_data[algos[1]][common_obstacles].values
        diff = np.mean(np.abs(values1 - values2))
        print(f"Success Rate - Average difference between {algos[0]} and {algos[1]}: {diff:.2f}%")

plt.tight_layout()
plt.savefig('perf_analysis6.png', dpi=300, bbox_inches='tight')
plt.show()

# Print bar graph differences (removed from PNG)
print("\n=== BAR GRAPH COMPARISONS ===")

# Success Rate by Algorithm
success_rates = df.groupby('algorithm')['success'].agg(['count', 'sum'])
success_rates['rate'] = success_rates['sum'] / success_rates['count'] * 100
if len(success_rates) == 2:
    algos = success_rates.index.tolist()
    diff = abs(success_rates.loc[algos[0], 'rate'] - success_rates.loc[algos[1], 'rate'])
    print(f"Success Rate - {algos[0]}: {success_rates.loc[algos[0], 'rate']:.2f}%, {algos[1]}: {success_rates.loc[algos[1], 'rate']:.2f}%")
    print(f"Success Rate Difference: {diff:.2f}%")

# Average Find Path Time by Algorithm
if not successful_df.empty:
    avg_find_time = successful_df.groupby('algorithm')['average_find_path_time_ns'].mean() / 1000000
    if len(avg_find_time) == 2:
        algos = avg_find_time.index.tolist()
        diff = abs(avg_find_time[algos[0]] - avg_find_time[algos[1]])
        print(f"Average Find Path Time - {algos[0]}: {avg_find_time[algos[0]]:.3f}ms, {algos[1]}: {avg_find_time[algos[1]]:.3f}ms")
        print(f"Find Path Time Difference: {diff:.3f}ms")

# Total Pathfinding Calls by Algorithm
if not successful_df.empty:
    avg_calls = successful_df.groupby('algorithm')['total_pathfinding_calls'].mean()
    if len(avg_calls) == 2:
        algos = avg_calls.index.tolist()
        diff = abs(avg_calls[algos[0]] - avg_calls[algos[1]])
        print(f"Average Pathfinding Calls - {algos[0]}: {avg_calls[algos[0]]:.2f}, {algos[1]}: {avg_calls[algos[1]]:.2f}")
        print(f"Pathfinding Calls Difference: {diff:.2f}")

# Route Efficiency by Algorithm
if not successful_df.empty:
    avg_efficiency = successful_df.groupby('algorithm')['route_efficiency'].mean()
    if len(avg_efficiency) == 2:
        algos = avg_efficiency.index.tolist()
        diff = abs(avg_efficiency[algos[0]] - avg_efficiency[algos[1]])
        print(f"Average Route Efficiency - {algos[0]}: {avg_efficiency[algos[0]]:.3f}, {algos[1]}: {avg_efficiency[algos[1]]:.3f}")
        print(f"Route Efficiency Difference: {diff:.3f}")

# Average Execution Time by Algorithm
if not successful_df.empty:
    avg_exec_time = successful_df.groupby('algorithm')['execution_time_ms'].mean()
    if len(avg_exec_time) == 2:
        algos = avg_exec_time.index.tolist()
        diff = abs(avg_exec_time[algos[0]] - avg_exec_time[algos[1]])
        print(f"Average Execution Time - {algos[0]}: {avg_exec_time[algos[0]]:.2f}ms, {algos[1]}: {avg_exec_time[algos[1]]:.2f}ms")
        print(f"Execution Time Difference: {diff:.2f}ms")

# Print summary statistics
print("\n=== ALGORITHM PERFORMANCE SUMMARY ===")
print("\nOverall Statistics:")
print(df.groupby('algorithm').agg({
    'success': ['count', 'sum', 'mean'],
    'total_moves': 'mean',
    'route_efficiency': 'mean',
    'execution_time_ms': 'mean',
    'average_find_path_time_ns': 'mean',
    'total_pathfinding_calls': 'mean'
}).round(3))

print("\nSuccessful Runs Only:")
if not successful_df.empty:
    print(successful_df.groupby('algorithm').agg({
        'total_moves': ['mean', 'std'],
        'route_efficiency': ['mean', 'std'],
        'execution_time_ms': ['mean', 'std'],
        'average_find_path_time_ns': ['mean', 'std'],
        'total_pathfinding_calls': ['mean', 'std']
    }).round(3))

# Additional detailed analysis
print("\n=== DETAILED PERFORMANCE ANALYSIS ===")
if not successful_df.empty:
    print("\nFind Path Time Analysis:")
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        avg_time = algo_data['average_find_path_time_ns'].mean() / 1000000
        print(f"{algorithm}: {avg_time:.3f} ms average find path time")
    
    print("\nExecution Time Analysis:")
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        avg_exec = algo_data['execution_time_ms'].mean()
        print(f"{algorithm}: {avg_exec:.3f} ms average execution time")
    
    print("\nPathfinding Calls Analysis:")
    for algorithm in successful_df['algorithm'].unique():
        algo_data = successful_df[successful_df['algorithm'] == algorithm]
        avg_calls = algo_data['total_pathfinding_calls'].mean()
        print(f"{algorithm}: {avg_calls:.2f} average pathfinding calls")
