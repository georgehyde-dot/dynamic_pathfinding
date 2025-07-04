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

#[derive(Debug, Clone)]
struct ObstacleGroup {
    positions: HashSet<Position>,
    cycles_remaining: usize,
}

pub struct Simulation {
    grid: Grid,
    agent: Agent,
    algorithm: Box<dyn PathfindingAlgorithm>,
    config: Config,
    optimal_path_length: usize,
    active_obstacle_groups: Vec<ObstacleGroup>,
    cycles_since_last_obstacle: usize,
    obstacle_cycle_interval: usize,
    obstacle_persistence_cycles: usize,
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
        let optimal_path_length = Self::calculate_optimal_path_static(&grid);
        
        if optimal_path_length == 0 {
            panic!("No valid path exists from start to goal! Try reducing num_walls.");
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
            config,
            optimal_path_length,
            active_obstacle_groups: Vec::new(),
            cycles_since_last_obstacle: 0,
            obstacle_cycle_interval: 5, // Place new obstacles every 5 cycles
            obstacle_persistence_cycles: 5, // Obstacles persist for 5 cycles
        }
    }

    pub fn run(&mut self) -> Statistics {
        let mut stats = Statistics::new(
            self.config.num_walls, 
            self.config.num_obstacles, 
            self.optimal_path_length
        );

        let mut total_iterations = 0;
        let max_iterations = self.grid.size * self.grid.size * 4; // Increased for dynamic obstacles

        // Print initial grid only if visualization is enabled
        if !self.config.no_visualization {
            self.clear_screen();
            println!("=== PATHFINDING SIMULATION ===");
            println!("Step: 0 | Moves: 0 | Active obstacle groups: 0");
            self.grid.print_grid(Some(self.agent.position));
            thread::sleep(Duration::from_millis(self.config.delay_ms));
        }

        while self.agent.position != self.grid.goal && total_iterations < max_iterations {
            // Update obstacle lifecycle
            self.update_obstacles();
            
            // Update agent's knowledge of obstacles
            self.agent.update_known_obstacles(&self.grid);
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
                    
                    // Only print and sleep if visualization is enabled
                    if !self.config.no_visualization {
                        self.clear_screen();
                        println!("=== PATHFINDING SIMULATION ===");
                        println!("Step: {} | Moves: {} | Active obstacle groups: {}", 
                                 total_iterations + 1, stats.total_moves, self.active_obstacle_groups.len());
                        println!("Agent position: ({}, {})", self.agent.position.x, self.agent.position.y);
                        println!("Goal position: ({}, {})", self.grid.goal.x, self.grid.goal.y);
                        println!("Cycles until next obstacles: {}", 
                                 self.obstacle_cycle_interval - self.cycles_since_last_obstacle);
                        
                        // Show obstacle group info
                        for (i, group) in self.active_obstacle_groups.iter().enumerate() {
                            println!("Obstacle group {}: {} obstacles, {} cycles remaining", 
                                     i + 1, group.positions.len(), group.cycles_remaining);
                        }
                        
                        // Show current path if available
                        if path.len() > 2 {
                            println!("Next few moves: {:?}", &path[1..path.len().min(4)]);
                        }
                        
                        self.grid.print_grid(Some(self.agent.position));
                        thread::sleep(Duration::from_millis(self.config.delay_ms));
                    }
                } else {
                    // Agent reached goal
                    break;
                }
            } else {
                // No path found - agent is stuck
                if !self.config.no_visualization {
                    self.clear_screen();
                    println!("=== PATHFINDING SIMULATION ===");
                    println!("WARNING: Agent got stuck at position {:?}", self.agent.position);
                    self.grid.print_grid(Some(self.agent.position));
                } else {
                    println!("Warning: Agent got stuck at position {:?}", self.agent.position);
                }
                break;
            }
            
            total_iterations += 1;
        }

        // Clean up any remaining obstacles
        self.clear_all_obstacles();

        // Final state - only print detailed info if visualization is enabled
        if !self.config.no_visualization {
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
        }

        if total_iterations >= max_iterations {
            println!("Warning: Simulation terminated due to iteration limit");
        }

        stats.calculate_efficiency();
        stats
    }

    /// Update obstacle lifecycle - place new obstacles and remove expired ones
    fn update_obstacles(&mut self) {
        // Increment cycle counter
        self.cycles_since_last_obstacle += 1;

        // Remove expired obstacles and decrement counters
        let mut expired_groups = Vec::new();
        for (i, group) in self.active_obstacle_groups.iter_mut().enumerate() {
            group.cycles_remaining = group.cycles_remaining.saturating_sub(1);
            if group.cycles_remaining == 0 {
                // Remove obstacles from grid
                for &pos in &group.positions {
                    self.grid.cells[pos.x][pos.y] = Cell::Empty;
                }
                expired_groups.push(i);
            }
        }

        // Remove expired groups (in reverse order to maintain indices)
        for &i in expired_groups.iter().rev() {
            self.active_obstacle_groups.remove(i);
        }

        // Place new obstacles if it's time
        if self.cycles_since_last_obstacle >= self.obstacle_cycle_interval {
            self.place_new_obstacle_group();
            self.cycles_since_last_obstacle = 0;
        }
    }

    /// Place a new group of obstacles
    fn place_new_obstacle_group(&mut self) {
        let mut new_group = ObstacleGroup {
            positions: HashSet::new(),
            cycles_remaining: self.obstacle_persistence_cycles,
        };

        let mut rng = rand::thread_rng();
        let mut attempts = 0;
        let max_attempts = self.config.num_obstacles * 10; // Prevent infinite loops

        while new_group.positions.len() < self.config.num_obstacles && attempts < max_attempts {
            let x = rng.gen_range(0..self.config.grid_size);
            let y = rng.gen_range(0..self.config.grid_size);
            let pos = Position { x, y };

            // Check if position is valid for obstacle placement
            if self.is_valid_obstacle_position(&pos) {
                new_group.positions.insert(pos);
                self.grid.cells[x][y] = Cell::Obstacle;
            }
            attempts += 1;
        }

        if !new_group.positions.is_empty() {
            self.active_obstacle_groups.push(new_group);
        }
    }

    /// Check if a position is valid for obstacle placement
    fn is_valid_obstacle_position(&self, pos: &Position) -> bool {
        // Can't place on start, goal, or agent position
        if *pos == self.grid.start || *pos == self.grid.goal || *pos == self.agent.position {
            return false;
        }

        // Can't place on walls or existing obstacles
        if self.grid.cells[pos.x][pos.y] != Cell::Empty {
            return false;
        }

        true
    }

    /// Clear all obstacles from the grid
    fn clear_all_obstacles(&mut self) {
        for group in &self.active_obstacle_groups {
            for &pos in &group.positions {
                self.grid.cells[pos.x][pos.y] = Cell::Empty;
            }
        }
        self.active_obstacle_groups.clear();
    }

    /// Clear the terminal screen (only used when visualization is enabled)
    fn clear_screen(&self) {
        print!("\x1B[2J\x1B[1;1H");
    }

    /// Calculate optimal path length with perfect knowledge (no obstacles, only walls)
    fn calculate_optimal_path_static(grid: &Grid) -> usize {
        let mut a_star = AStar::new();
        if let Some(path) = a_star.find_path(grid, grid.start, grid.goal, &HashSet::new()) {
            path.len().saturating_sub(1)
        } else {
            0
        }
    }
}
