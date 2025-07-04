# Pathfinding in Simulated Worlds

This repository contains the Rust implementation for the research proposal "Pathfinding in Simulated Worlds." It provides a simulation environment to compare the performance of A* and D* Lite pathfinding algorithms in a 2D grid with static walls and dynamic obstacles.

## Features

- 2D grid-based simulation environment with visual representation
- Implementation of A* and D* Lite pathfinding algorithms
- Configurable grid size, number of walls, and obstacles
- Agent with a limited field of view that discovers obstacles dynamically
- Real-time visualization with customizable delay
- Detailed statistics tracking including route efficiency analysis
- Performance mode for large-scale testing without visualization

## Building and Running

To build and run the simulation, you will need to have Rust and Cargo installed.

1.  **Clone the repository:**
      ```bash
      git clone <repository-url>
      cd <repository-directory>
      ```

2.  **Build the project:**
      ```bash
      cargo build --release
      ```

3.  **Run the simulation with default settings:**
      ```bash
      cargo run --release
      ```

## Command-Line Arguments

The simulation accepts several command-line arguments to customize the behavior:

### Core Configuration
- `--grid-size <SIZE>`: The size of the square grid (default: 20)
    - Creates a SIZE×SIZE grid
    - Recommended: 10-50 for visualization, up to 250+ for performance testing

- `--num-walls <COUNT>`: The number of static walls to place in the grid (default: 50)
    - Walls are permanent obstacles placed at grid initialization
    - Should be less than grid-size² to ensure pathfinding is possible

- `--num-obstacles <COUNT>`: The number of dynamic obstacles to place during simulation (default: 10)
    - Obstacles appear one per simulation step as the agent moves
    - Agent must discover and navigate around these dynamically

- `--algorithm <ALGORITHM>`: The pathfinding algorithm to use (default: "a_star")
    - Options: `a_star` or `d_star_lite`
    - A* recalculates the entire path when obstacles are discovered
    - D* Lite incrementally updates the path for better performance with dynamic obstacles

### Visualization Options
- `--delay-ms <MILLISECONDS>`: Delay between simulation steps in milliseconds (default: 50)
    - Controls animation speed when visualization is enabled
    - Lower values = faster animation, higher values = slower/easier to follow

- `--no-visualization`: Disable visual output and run in performance mode (default: false)
    - Skips all grid printing and delays for maximum performance
    - Useful for large-scale testing and benchmarking
    - Still shows initial setup and final results

## Usage Examples

### Basic Examples
```bash
# Run with default settings (20×20 grid, A*, with visualization)
cargo run --release

# Run a small simulation that's easy to follow
cargo run --release -- --grid-size 15 --num-walls 30 --num-obstacles 5

# Test D* Lite algorithm
cargo run --release -- --algorithm d_star_lite --grid-size 15
```

### Visualization Examples
```bash
# Slow animation for detailed observation
cargo run --release -- --delay-ms 200 --grid-size 12 --num-obstacles 3

# Fast animation
cargo run --release -- --delay-ms 25 --grid-size 20

# No visualization - performance mode
cargo run --release -- --no-visualization --grid-size 50 --num-walls 200
```

### Performance Testing Examples
```bash
# Large grid performance test
cargo run --release -- --no-visualization --grid-size 100 --num-walls 500 --num-obstacles 50

# Compare algorithms on identical large setup
cargo run --release -- --no-visualization --algorithm a_star --grid-size 75 --num-walls 300 --num-obstacles 100
cargo run --release -- --no-visualization --algorithm d_star_lite --grid-size 75 --num-walls 300 --num-obstacles 100

# Stress test with many obstacles
cargo run --release -- --no-visualization --grid-size 50 --num-walls 100 --num-obstacles 200
```

### Research/Comparison Examples
```bash
# Small controlled environment for algorithm comparison
cargo run --release -- --algorithm a_star --grid-size 20 --num-walls 50 --num-obstacles 10
cargo run --release -- --algorithm d_star_lite --grid-size 20 --num-walls 50 --num-obstacles 10

# Medium complexity scenario
cargo run --release -- --algorithm a_star --grid-size 30 --num-walls 100 --num-obstacles 25 --delay-ms 30

# High obstacle density test
cargo run --release -- --no-visualization --grid-size 40 --num-walls 80 --num-obstacles 150
```

## Understanding the Output

### During Simulation (with visualization)
```
=== PATHFINDING SIMULATION ===
Step: 15 | Moves: 12 | Obstacles placed: 3
Agent position: (8, 7)
Goal position: (18, 19)
Next few moves: [(9, 7), (10, 7), (11, 7)]

Legend: S=Start, G=Goal, A=Agent, #=Wall, O=Obstacle, .=Empty
     0 1 2 3 4 5 6 7 8 9101112131415161718192
   0 S . . # . . . . . . . . . . . . . . . . 
   1 . . . . . . . . . . . . . . . . . . . . 
   2 . . # . . . . . . . . . . . . . . . . . 
...
```

### Final Results
```
=== FINAL RESULTS ===
Total Moves: 45
Optimal Path Length: 32
Number of Walls: 50
Number of Obstacles: 10
Route Efficiency: 0.711
Efficiency Percentage: 71.1%
Extra moves due to obstacles/limited vision: 13
```

### Statistics Explanation
- **Total Moves**: Actual number of steps the agent took to reach the goal
- **Optimal Path Length**: Theoretical minimum steps with perfect knowledge (walls only, no obstacles)
- **Route Efficiency**: Ratio of optimal path length to actual moves (higher is better)
- **Efficiency Percentage**: Route efficiency as a percentage (100% = perfect efficiency)
- **Extra Moves**: Additional steps caused by obstacles and limited field of view

## Project Structure

- `src/main.rs`: The main entry point and command-line argument processing
- `src/lib.rs`: Defines public modules for shared use in the project
- `src/config.rs`: Command-line configuration structure using clap
- `src/grid.rs`: Core data structures for the grid, positions, and cells
- `src/agent.rs`: The agent that navigates the grid with limited field of view
- `src/simulation.rs`: The main simulation logic and visualization
- `src/statistics.rs`: Tracks and calculates simulation statistics and efficiency
- `src/algorithms/`: Contains the pathfinding algorithm implementations
    - `a_star.rs`: A* algorithm implementation using the pathfinding crate
    - `d_star_lite.rs`: D* Lite algorithm implementation for dynamic replanning
    - `common.rs`: Common trait interface for pathfinding algorithms

## Algorithm Comparison

### A* Algorithm
- **Best for**: Static environments, simple scenarios
- **Behavior**: Recalculates entire path when obstacles are discovered
- **Performance**: Fast for small grids, can be slower with many dynamic obstacles
- **Use case**: Baseline comparison, environments with few changes

### D* Lite Algorithm
- **Best for**: Dynamic environments with frequent obstacle changes
- **Behavior**: Incrementally updates existing path when obstacles are discovered
- **Performance**: More efficient with many dynamic obstacles
- **Use case**: Real-world scenarios with changing environments

## Tips for Effective Testing

1. **Start Small**: Use `--grid-size 15` or smaller for initial testing and visualization
2. **Use Performance Mode**: Add `--no-visualization` for large-scale testing
3. **Balance Obstacles**: Too many walls can make paths impossible; too few make testing unrealistic
4. **Compare Algorithms**: Run identical scenarios with both algorithms to see differences
5. **Adjust Field of View**: Modify `field_of_view` in `src/agent.rs` to test different visibility scenarios
6. **Monitor Efficiency**: Route efficiency below 50% indicates significant pathfinding challenges

## Troubleshooting

- **"No valid path exists"**: Reduce `--num-walls` or increase `--grid-size`
- **Agent gets stuck**: Usually caused by obstacles blocking all paths; try fewer obstacles
- **Simulation too fast/slow**: Adjust `--delay-ms` or use `--no-visualization`
- **Performance issues**: Use `--no-visualization` for large grids (50+)
