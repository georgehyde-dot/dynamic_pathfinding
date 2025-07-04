# Pathfinding in Simulated Worlds

This repository contains the Rust implementation for the research proposal "Pathfinding in Simulated Worlds." It provides a simulation environment to compare the performance of A* and D* Lite pathfinding algorithms in a 2D grid with static walls and dynamic obstacles.

## Features

- 2D grid-based simulation environment.
- Implementation of A* and D* Lite pathfinding algorithms.
- Configurable grid size, number of walls, and obstacles.
- Agent with a limited field of view.
- Detailed statistics tracking for each simulation run.

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

3.  **Run the simulation:**
    ```bash
    cargo run --release -- --grid-size 250 --num-walls 500 --num-obstacles 100 --algorithm a_star
    ```

## Command-Line Arguments

- `--grid-size`: The size of the square grid (e.g., 250 for a 250x250 grid).
- `--num-walls`: The number of static walls to place in the grid.
- `--num-obstacles`: The number of dynamic obstacles to place in the grid.
- `--algorithm`: The pathfinding algorithm to use (`a_star` or `d_star_lite`).

## Project Structure

- `src/main.rs`: The main entry point of the application.
- `src/lib.rs`: Defines public modules for shared use in the project
- `src/config.rs`: Defines the simulation configuration.
- `src/grid.rs`: Core data structures for the grid and its components.
- `src/agent.rs`: The agent that navigates the grid.
- `src/simulation.rs`: The main simulation logic.
- `src/statistics.rs`: Tracks and calculates simulation statistics.
- `src/algorithms/`: Contains the pathfinding algorithm implementations.
  - `a_star.rs`: A* algorithm implementation.
  - `d_star_lite.rs`: D* Lite algorithm implementation.
  - `common.rs`: A common trait for pathfinding algorithms.
