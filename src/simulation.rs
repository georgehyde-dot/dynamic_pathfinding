use crate::agent::Agent;
use crate::algorithms::a_star::AStar;
use crate::algorithms::common::PathfindingAlgorithm;
use crate::algorithms::d_star_lite::DStarLite;
use crate::config::Config;
use crate::grid::{Cell, Grid, Position};
use crate::statistics::Statistics;
use rand::Rng;
use std::collections::HashSet;
use std::thread;
use std::time::Duration;

pub struct Simulation {
    grid: Grid,
    agent: Agent,
    algorithm: Box<dyn PathfindingAlgorithm>,
    obstacles: Vec<HashSet<Position>>,
    config: Config,
    optimal_path_length: usize,
}

impl Simulation {
    pub fn new(config: Config) -> Self {
        let grid = Grid::new(config.grid_size, config.num_walls);
        let agent = Agent::new(grid.start);

        let algorithm: Box<dyn PathfindingAlgorithm> = if config.algorithm == "a_star" {
            Box::new(AStar::new())
        } else {
            Box::new(DStarLite::new(grid.start, grid.goal))
        };

        // Calculate optimal path BEFORE placing any obstacles
        // This represents the theoretical best path with perfect knowledge of walls only
        let optimal_path_length = Self::calculate_optimal_path_static(&grid);
        
        if optimal_path_length == 0 {
            panic!("No valid path exists from start to goal! Try reducing num_walls.");
        }

        // Generate obstacle iterations - ensure obstacles are actually placed
        let mut obstacles = Vec::new();
        let mut rng = rand::thread_rng();
        for _ in 0..config.num_obstacles {
            let mut obstacle_set = HashSet::new();
            let mut attempts = 0;
            
            // Try to place an obstacle, with fallback
            while obstacle_set.is_empty() && attempts < 10 {
                let x = rng.gen_range(0..config.grid_size);
                let y = rng.gen_range(0..config.grid_size);
                let pos = Position { x, y };
                
                if grid.cells[x][y] == Cell::Empty && pos != grid.start && pos != grid.goal {
                    obstacle_set.insert(pos);
                }
                attempts += 1;
            }
            obstacles.push(obstacle_set);
        }

        // Validate that a path exists before starting simulation
        let mut temp_astar = AStar::new();
        if temp_astar.find_path(&grid, grid.start, grid.goal, &HashSet::new()).is_none() {
            panic!("No valid path exists from start to goal! Try reducing num_walls.");
        }

        Simulation {
            grid,
            agent,
            algorithm,
            obstacles,
            config,
            optimal_path_length,
        }
    }

    pub fn run(&mut self) -> Statistics {
        let mut stats = Statistics::new(
            self.config.num_walls, 
            self.config.num_obstacles, 
            self.optimal_path_length
        );

        let mut obstacle_iter = 0;
        let mut total_iterations = 0;
        let max_iterations = self.grid.size * self.grid.size * 2; // Prevent infinite loops

        // Print initial grid
        self.clear_screen();
        println!("=== PATHFINDING SIMULATION ===");
        println!("Step: 0 | Moves: 0 | Obstacles placed: 0");
        self.grid.print_grid(Some(self.agent.position));
        thread::sleep(Duration::from_millis(50));

        while self.agent.position != self.grid.goal && total_iterations < max_iterations {
            // Place new obstacles
            if obstacle_iter < self.obstacles.len() {
                for &obstacle_pos in &self.obstacles[obstacle_iter] {
                    self.grid.cells[obstacle_pos.x][obstacle_pos.y] = Cell::Obstacle;
                }
                obstacle_iter += 1;
            }

            self.agent.observe(&self.grid);

            let path = self.algorithm.find_path(
                &self.grid,
                self.agent.position,
                self.grid.goal,
                &self.agent.known_obstacles,
            );

            if let Some(path) = path {
                if path.len() > 1 {
                    let next_pos = path[1];
                    self.agent.move_to(next_pos);
                    stats.total_moves += 1;
                    
                    // Clear screen and print updated grid
                    self.clear_screen();
                    println!("=== PATHFINDING SIMULATION ===");
                    println!("Step: {} | Moves: {} | Obstacles placed: {}", 
                             total_iterations + 1, stats.total_moves, obstacle_iter);
                    println!("Agent position: ({}, {})", self.agent.position.x, self.agent.position.y);
                    println!("Goal position: ({}, {})", self.grid.goal.x, self.grid.goal.y);
                    
                    // Show current path if available
                    if path.len() > 2 {
                        println!("Next few moves: {:?}", &path[1..path.len().min(4)]);
                    }
                    
                    self.grid.print_grid(Some(self.agent.position));
                    
                    // Sleep for 50ms
                    thread::sleep(Duration::from_millis(50));
                } else {
                    // Agent reached goal
                    break;
                }
            } else {
                // No path found - agent is stuck
                self.clear_screen();
                println!("=== PATHFINDING SIMULATION ===");
                println!("WARNING: Agent got stuck at position {:?}", self.agent.position);
                self.grid.print_grid(Some(self.agent.position));
                break;
            }
            
            total_iterations += 1;
        }

        // Final state
        self.clear_screen();
        println!("=== SIMULATION COMPLETE ===");
        if self.agent.position == self.grid.goal {
            println!("SUCCESS: Agent reached the goal!");
        } else {
            println!("FAILED: Agent did not reach the goal");
        }
        println!("Final position: ({}, {})", self.agent.position.x, self.agent.position.y);
        println!("Total steps: {} | Total moves: {}", total_iterations, stats.total_moves);
        self.grid.print_grid(Some(self.agent.position));

        if total_iterations >= max_iterations {
            println!("Warning: Simulation terminated due to iteration limit");
        }

        stats.calculate_efficiency();
        stats
    }

    /// Clear the terminal screen
    fn clear_screen(&self) {
        // ANSI escape code to clear screen and move cursor to top-left
        print!("\x1B[2J\x1B[1;1H");
    }

    /// Calculate optimal path length with perfect knowledge (no obstacles, only walls)
    fn calculate_optimal_path_static(grid: &Grid) -> usize {
        let mut a_star = AStar::new();
        if let Some(path) = a_star.find_path(grid, grid.start, grid.goal, &HashSet::new()) {
            // Return number of moves (path length - 1)
            path.len().saturating_sub(1)
        } else {
            0
        }
    }
}
