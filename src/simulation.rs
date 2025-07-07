use crate::agent::Agent;
use crate::algorithms::a_star::AStar;
use crate::algorithms::common::PathfindingAlgorithm;
use crate::algorithms::d_star_lite::DStarLite;

use crate::algorithms::hybrid_a_star_d_star::HybridAStarDStar;
use crate::config::Config;
use crate::grid::{Cell, Grid, Position};
use crate::statistics::{Statistics, AlgorithmStats};
use rand::{Rng, SeedableRng};
use std::collections::HashSet;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct ObstacleGroup {
    positions: HashSet<Position>,
    cycles_remaining: usize,
}

#[derive(Debug, Clone)]
pub struct EnvironmentSetup {
    pub grid_size: usize,
    pub start: Position,
    pub goal: Position,
    pub walls: HashSet<Position>,

    pub obstacle_timeline: Vec<HashSet<Position>>,
    pub obstacle_cycle_interval: usize,
    pub obstacle_persistence_cycles: usize,
}

impl EnvironmentSetup {

    pub fn generate(config: &Config, seed: Option<u64>) -> Self {

        let mut rng = if let Some(seed) = seed {
            rand::rngs::StdRng::seed_from_u64(seed)
        } else {
            rand::rngs::StdRng::from_entropy()
        };


        let start = Position { 
            x: rng.gen_range(0..config.grid_size/2), 
            y: rng.gen_range(0..config.grid_size/2) 
        };
        let goal = Position { 
            x: rng.gen_range(config.grid_size/2..config.grid_size), 
            y: rng.gen_range(config.grid_size/2..config.grid_size) 
        };


        let mut walls = HashSet::new();
        let mut walls_placed = 0;
        let mut attempts = 0;
        while walls_placed < config.num_walls && attempts < config.num_walls * 3 {
            let x = rng.gen_range(0..config.grid_size);
            let y = rng.gen_range(0..config.grid_size);
            let pos = Position { x, y };
            
            if pos != start && pos != goal && !walls.contains(&pos) {
                walls.insert(pos);
                walls_placed += 1;
            }
            attempts += 1;
        }


        let obstacle_cycle_interval = 5;
        let obstacle_persistence_cycles = 5;

        let max_cycles = config.grid_size * config.grid_size;
        let num_obstacle_cycles = max_cycles / obstacle_cycle_interval;
        
        let mut obstacle_timeline = Vec::new();
        

        for _cycle in 0..num_obstacle_cycles {
            let mut obstacle_group = HashSet::new();
            let mut attempts = 0;
            let max_attempts = config.num_obstacles * 10;

            while obstacle_group.len() < config.num_obstacles && attempts < max_attempts {
                let x = rng.gen_range(0..config.grid_size);
                let y = rng.gen_range(0..config.grid_size);
                let pos = Position { x, y };


                if pos != start && pos != goal && 
                   !walls.contains(&pos) && 
                   !obstacle_group.contains(&pos) {
                    obstacle_group.insert(pos);
                }
                attempts += 1;
            }
            
            obstacle_timeline.push(obstacle_group);
        }




        EnvironmentSetup {
            grid_size: config.grid_size,
            start,
            goal,
            walls,
            obstacle_timeline,
            obstacle_cycle_interval,
            obstacle_persistence_cycles,
        }
    }


    pub fn create_grid(&self) -> Grid {
        let mut cells = vec![vec![Cell::Empty; self.grid_size]; self.grid_size];
        

        for &wall_pos in &self.walls {
            cells[wall_pos.x][wall_pos.y] = Cell::Wall;
        }

        Grid {
            size: self.grid_size,
            cells,
            start: self.start,
            goal: self.goal,
        }
    }
}

#[derive(Debug, Clone)]
pub struct AlgorithmResult {
    pub name: String,
    pub statistics: Statistics,
    pub success: bool,
    pub final_position: Position,
    pub algorithm_stats: AlgorithmStats,
    pub timing_data: TimingData,
}

pub struct AlgorithmRunner {
    pub name: String,
    pub create_algorithm: Box<dyn Fn(Position, Position) -> Box<dyn PathfindingAlgorithm>>,
}

impl AlgorithmRunner {
    pub fn new<F>(name: &str, create_fn: F) -> Self 
    where 
        F: Fn(Position, Position) -> Box<dyn PathfindingAlgorithm> + 'static,
    {
        AlgorithmRunner {
            name: name.to_string(),
            create_algorithm: Box::new(create_fn),
        }
    }
}

pub struct Simulation {
    pub grid: Grid,
    pub agent: Agent,
    algorithm: Box<dyn PathfindingAlgorithm>,
    config: Config,
    optimal_path_length: usize,
    environment: EnvironmentSetup,
    active_obstacle_groups: Vec<ObstacleGroup>,
    cycles_since_last_obstacle: usize,
    current_obstacle_cycle: usize,
}

impl Simulation {
    pub fn new(config: Config) -> Result<Self, String> {
        Self::new_with_environment(config, None)
    }

    pub fn new_with_environment(config: Config, environment: Option<EnvironmentSetup>) -> Result<Self, String> {
        let environment = environment.unwrap_or_else(|| EnvironmentSetup::generate(&config, None));
        let grid = environment.create_grid();
        let agent = Agent::new(grid.start);




        let algorithm: Box<dyn PathfindingAlgorithm> = match config.algorithm.as_str() {
            "a_star" => Box::new(AStar::new()),
            "d_star_lite" => Box::new(DStarLite::new(grid.start, grid.goal)),

            "hybrid" => Box::new(HybridAStarDStar::new(grid.start, grid.goal)),
            _ => return Err(format!("Unknown algorithm: '{}'", config.algorithm)),
        };


        let optimal_path_length = Self::calculate_optimal_path_with_astar(&grid);
        
        if optimal_path_length == 0 {
            return Err(format!("No valid path exists from start {:?} to goal {:?}! Grid has {} walls.", 
                              grid.start, grid.goal, 
                              grid.cells.iter().flatten().filter(|&cell| *cell == Cell::Wall).count()));
        }

        Ok(Simulation {
            grid,
            agent,
            algorithm,
            config,
            optimal_path_length,
            environment,
            active_obstacle_groups: Vec::new(),
            cycles_since_last_obstacle: 0,
            current_obstacle_cycle: 0,
        })
    }

    pub fn run(&mut self) -> (Statistics, AlgorithmStats, TimingData) {
        let mut stats = Statistics::new(
            self.config.num_walls, 
            self.config.num_obstacles, 
            self.optimal_path_length
        );

        let mut total_iterations = 0;
        let max_iterations = self.grid.size * self.grid.size * 4;
        
        // Track timing data
        let mut timing_data = TimingData::new();
        
        // Track stuck attempts
        let mut stuck_attempts = 0;
        const MAX_STUCK_ATTEMPTS: usize = 5;
        
        // Print initial grid only if visualization is enabled
        if !self.config.no_visualization {
            self.clear_screen();
            println!("=== PATHFINDING SIMULATION ===");
            println!("Algorithm: {} | Step: 0 | Moves: 0 | Active obstacle groups: 0", self.config.algorithm);
            println!("Optimal path length (A*): {}", self.optimal_path_length);
            self.grid.print_grid(Some(self.agent.position));
            thread::sleep(Duration::from_millis(self.config.delay_ms));
        }

        // Calculate initial path
        let initial_path = self.algorithm.find_path(
            &self.grid,
            self.agent.position,
            self.grid.goal,
            &self.agent.known_obstacles,
        );
        
        if let Some(path) = initial_path {
            self.agent.set_path(path);
        } else {
            // No initial path found
            return (stats, self.get_algorithm_stats(), timing_data);
        }

        while self.agent.position != self.grid.goal && total_iterations < max_iterations {
            // Update obstacle lifecycle using pre-generated timeline
            let obstacles_changed = self.update_obstacles_from_timeline();
            
            // Agent observes environment
            let observe_start = Instant::now();
            self.agent.observe(&self.grid);
            let observe_duration = observe_start.elapsed();
            timing_data.observe_times.push(observe_duration);
            
            // Check if path needs recalculation
            let needs_recalc = self.agent.path_needs_recalculation(&self.grid) || 
                              self.agent.is_path_blocked(&self.grid) ||
                              obstacles_changed;
            
            if needs_recalc {
                if !self.config.no_visualization {
                    println!("Path blocked or environment changed - recalculating...");
                }
                
                // Notify algorithm of environment changes (for incremental algorithms)
                self.algorithm.update_environment(&self.grid, &self.agent.known_obstacles);
                
                // Recalculate path
                let find_path_start = Instant::now();
                let new_path = self.algorithm.find_path(
                    &self.grid,
                    self.agent.position,
                    self.grid.goal,
                    &self.agent.known_obstacles,
                );
                let find_path_duration = find_path_start.elapsed();
                timing_data.find_path_times.push(find_path_duration);
                
                if let Some(path) = new_path {
                    self.agent.set_path(path);
                    stuck_attempts = 0; // Reset stuck counter
                    
                    if !self.config.no_visualization {
                        println!("New path found with {} steps", self.agent.get_current_path().unwrap().len());
                    }
                } else {
                    // No path found - agent is stuck
                    stuck_attempts += 1;
                    
                    if stuck_attempts <= MAX_STUCK_ATTEMPTS {
                        stats.total_moves += 1; // Count waiting as a move
                        
                        if !self.config.no_visualization {
                            println!("No path found - waiting... (attempt {}/{})", stuck_attempts, MAX_STUCK_ATTEMPTS);
                        }
                    } else {
                        if !self.config.no_visualization {
                            println!("FAILURE: Agent permanently stuck after {} attempts", MAX_STUCK_ATTEMPTS);
                        }
                        break;
                    }
                }
            }
            
            // Follow current path (only if we have a valid path and aren't stuck)
            if stuck_attempts == 0 {
                if let Some(next_pos) = self.agent.get_next_step() {
                    self.agent.move_to(next_pos);
                    stats.total_moves += 1;
                    
                    if !self.config.no_visualization {
                        self.clear_screen();
                        println!("=== PATHFINDING SIMULATION ===");
                        println!("Algorithm: {} | Step: {} | Moves: {} | Active obstacle groups: {}", 
                                 self.config.algorithm, total_iterations + 1, stats.total_moves, self.active_obstacle_groups.len());
                        
                        let (path_progress, path_total) = self.agent.get_path_progress();
                        println!("Agent position: ({}, {}) | Path progress: {}/{}", 
                                 self.agent.position.x, self.agent.position.y, path_progress, path_total);
                        println!("Goal position: ({}, {})", self.grid.goal.x, self.grid.goal.y);
                        println!("Original optimal path (A*): {}", self.optimal_path_length);
                        println!("Obstacle cycle: {} | Cycles until next: {}", 
                                 self.current_obstacle_cycle,
                                 self.environment.obstacle_cycle_interval - self.cycles_since_last_obstacle);
                        
                        // Show timing info
                        if !timing_data.observe_times.is_empty() {
                            println!("Last observe: {:.2?} | Recalculations: {}", 
                                     timing_data.observe_times.last().unwrap(),
                                     timing_data.find_path_times.len());
                        }
                        
                        if !timing_data.find_path_times.is_empty() {
                            println!("Last find_path: {:.2?} | Avg find_path: {:.2?}", 
                                     timing_data.find_path_times.last().unwrap(),
                                     timing_data.average_find_path_time());
                        }
                        
                        // Show obstacle group info
                        for (i, group) in self.active_obstacle_groups.iter().enumerate() {
                            println!("Obstacle group {}: {} obstacles, {} cycles remaining", 
                                     i + 1, group.positions.len(), group.cycles_remaining);
                        }
                        
                        // Show next few moves in current path
                        if let Some(path) = self.agent.get_current_path() {
                            let (current_idx, _) = self.agent.get_path_progress();
                            if current_idx + 1 < path.len() {
                                let next_moves: Vec<_> = path.iter()
                                    .skip(current_idx + 1)
                                    .take(3)
                                    .collect();
                                println!("Next moves: {:?}", next_moves);
                            }
                        }
                        
                        self.grid.print_grid(Some(self.agent.position));
                        thread::sleep(Duration::from_millis(self.config.delay_ms));
                    }
                } else {
                    // Reached end of path - should be at goal
                    if !self.agent.is_at_goal(self.grid.goal) {
                        if !self.config.no_visualization {
                            println!("Warning: Reached end of path but not at goal!");
                        }
                        // Force recalculation
                        self.agent.clear_path();
                    }
                    break;
                }
            }
            
            total_iterations += 1;
            if total_iterations >= max_iterations {
                if !self.config.no_visualization {
                    println!("Reached max iterations, stopping simulation");
                }
                break;
            }
        }

        // Clean up any remaining obstacles
        self.clear_all_obstacles();

        // Final state
        if !self.config.no_visualization {
            self.clear_screen();
            println!("=== SIMULATION COMPLETE ===");
            if self.agent.is_at_goal(self.grid.goal) {
                println!("SUCCESS: Agent reached the goal!");
            } else {
                println!("FAILED: Agent did not reach the goal");
            }
            println!("Algorithm: {}", self.config.algorithm);
            println!("Final position: ({}, {})", self.agent.position.x, self.agent.position.y);
            println!("Total steps: {} | Total moves: {}", total_iterations, stats.total_moves);
            println!("Original optimal path (A*): {}", self.optimal_path_length);
            println!("Path recalculations: {}", timing_data.find_path_times.len());
            
            // Show timing summary
            println!("Average observe time: {:.2?}", timing_data.average_observe_time());
            println!("Average find_path time: {:.2?}", timing_data.average_find_path_time());
            
            // Calculate final optimal path
            let final_optimal_length = Self::calculate_optimal_path_with_astar(&self.grid);
            println!("Final optimal path (A*): {}", final_optimal_length);
            
            self.grid.print_grid(Some(self.agent.position));
        }

        stats.calculate_efficiency();
        (stats, self.get_algorithm_stats(), timing_data)
    }

    /// Get algorithm statistics based on algorithm type
    fn get_algorithm_stats(&self) -> AlgorithmStats {
        let path_calculations = self.get_path_calculation_count();
        
        match self.config.algorithm.as_str() {
            "a_star" => AlgorithmStats::AStar(path_calculations),
            "d_star_lite" => AlgorithmStats::DStarLite(path_calculations),
            "hybrid" => {
                let (a_star_calls, d_star_calls) = self.algorithm.get_usage_stats();
                AlgorithmStats::Hybrid { a_star_calls, d_star_calls }
            },
            _ => AlgorithmStats::AStar(path_calculations),
        }
    }

    /// Get total number of path calculations performed
    fn get_path_calculation_count(&self) -> usize {
        // This should be tracked by timing_data.find_path_times.len()
        // but for hybrid algorithms, we need to use their internal counters
        match self.config.algorithm.as_str() {
            "hybrid" => {
                let (a_star_calls, d_star_calls) = self.algorithm.get_usage_stats();
                a_star_calls + d_star_calls
            },
            _ => {
                // For non-hybrid algorithms, count is the number of find_path calls
                // This will be properly tracked by timing_data, but we need to access it
                // For now, return 0 and let the caller use timing_data.find_path_times.len()
                0
            }
        }
    }

    /// Update obstacles using the pre-generated timeline
    /// Returns true if obstacles changed
    fn update_obstacles_from_timeline(&mut self) -> bool {
        let mut obstacles_changed = false;
        
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
                obstacles_changed = true;
            }
        }

        // Remove expired groups (in reverse order to maintain indices)
        for &i in expired_groups.iter().rev() {
            self.active_obstacle_groups.remove(i);
        }

        // Place new obstacles if it's time and we have more in the timeline
        if self.cycles_since_last_obstacle >= self.environment.obstacle_cycle_interval {
            if self.current_obstacle_cycle < self.environment.obstacle_timeline.len() {
                obstacles_changed = self.place_obstacle_group_from_timeline() || obstacles_changed;
                self.current_obstacle_cycle += 1;
            }
            self.cycles_since_last_obstacle = 0;
        }

        obstacles_changed
    }

    /// Place obstacles from the pre-generated timeline
    /// Returns true if obstacles were placed
    fn place_obstacle_group_from_timeline(&mut self) -> bool {
        let obstacle_positions = &self.environment.obstacle_timeline[self.current_obstacle_cycle];
        
        let mut new_group = ObstacleGroup {
            positions: HashSet::new(),
            cycles_remaining: self.environment.obstacle_persistence_cycles,
        };

        // Place obstacles from the timeline
        for &pos in obstacle_positions {
            // Double-check that position is still valid (not occupied by agent)
            if self.is_valid_obstacle_position(&pos) {
                new_group.positions.insert(pos);
                self.grid.cells[pos.x][pos.y] = Cell::Obstacle;
            }
        }

        if !new_group.positions.is_empty() {
            self.active_obstacle_groups.push(new_group);
            true
        } else {
            false
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

    /// Calculate optimal path length using A* with current grid state
    fn calculate_optimal_path_with_astar(grid: &Grid) -> usize {
        let mut a_star = AStar::new();
        if let Some(path) = a_star.find_path(grid, grid.start, grid.goal, &HashSet::new()) {
            path.len().saturating_sub(1)
        } else {
            0
        }
    }

    /// Run all algorithms and compare results
    pub fn run_all_algorithms(config: Config) -> Result<Vec<AlgorithmResult>, String> {
        // Generate a random seed for this run, but use it consistently across all algorithms
        let run_seed = rand::random::<u64>();
        let environment = EnvironmentSetup::generate(&config, Some(run_seed));
        
        // Define available algorithms
        let algorithms = [
            AlgorithmRunner::new("a_star", |_start, _goal| Box::new(AStar::new())),
            AlgorithmRunner::new("d_star_lite", |start, goal| Box::new(DStarLite::new(start, goal))),
            AlgorithmRunner::new("hybrid", |start, goal| Box::new(HybridAStarDStar::new(start, goal))),
        ];

        let mut results = Vec::new();

        // Create grid for all algorithms to use
        let grid = environment.create_grid();

        // Calculate optimal path using A* (no obstacles, only walls)
        let optimal_path_length = Self::calculate_optimal_path_with_astar(&grid);
        
        if optimal_path_length == 0 {
            return Err(format!("No valid path exists from start {:?} to goal {:?}! Grid has {} walls.", 
                              grid.start, grid.goal, 
                              grid.cells.iter().flatten().filter(|&cell| *cell == Cell::Wall).count()));
        }

        for algorithm_runner in algorithms.iter() {
            // Create a new config for this algorithm run (no visualization)
            let mut algorithm_config = config.clone();
            algorithm_config.no_visualization = true;
            algorithm_config.algorithm = algorithm_runner.name.clone();

            // Create simulation with the shared environment
            match Self::new_with_environment_and_algorithm(
                algorithm_config,
                environment.clone(),
                (algorithm_runner.create_algorithm)(environment.start, environment.goal),
                optimal_path_length,
                &grid
            ) {
                Ok(mut simulation) => {
                    // Run the simulation
                    let (statistics, algorithm_stats, timing_data) = simulation.run();
                    let success = simulation.agent.is_at_goal(simulation.grid.goal);
                    let final_position = simulation.agent.position;

                    results.push(AlgorithmResult {
                        name: algorithm_runner.name.clone(),
                        statistics,
                        algorithm_stats,
                        timing_data,
                        success,
                        final_position,
                    });
                }
                Err(e) => {
                    // Handle simulation creation failure
                    if !config.quiet {
                        println!("Failed to create simulation for {}: {}", algorithm_runner.name, e);
                    }
                    
                    let failed_result = AlgorithmResult {
                        name: algorithm_runner.name.clone(),
                        statistics: Statistics::new(config.num_walls, config.num_obstacles, 0),
                        algorithm_stats: AlgorithmStats::AStar(0),
                        timing_data: TimingData::new(),
                        success: false,
                        final_position: grid.start,
                    };
                    results.push(failed_result);
                }
            }
        }

        Ok(results)
    }

    /// Create simulation with specific environment and algorithm
    pub fn new_with_environment_and_algorithm(
        config: Config, 
        environment: EnvironmentSetup, 
        algorithm: Box<dyn PathfindingAlgorithm>,
        optimal_path_length: usize,
        grid: &Grid
    ) -> Result<Self, String> {
        let agent = Agent::new(grid.start);
        let sim_grid = grid.clone();

        Ok(Simulation {
            grid: sim_grid,
            agent,
            algorithm,
            config,
            optimal_path_length,
            environment,
            active_obstacle_groups: Vec::new(),
            cycles_since_last_obstacle: 0,
            current_obstacle_cycle: 0,
        })
    }

    /// Print comparison results in a nice table format
    pub fn print_comparison_results(results: &[AlgorithmResult]) {
        println!("\n=== ALGORITHM COMPARISON RESULTS ===");
        println!();
        
        // Print header
        println!("{:<15} {:<8} {:<8} {:<8} {:<12} {:<15} {:<15} {:<15} {:<15} {:<20}", 
                 "Algorithm", "Success", "Moves", "Optimal", "Efficiency", "Avg Observe", "Avg Find Path", "Path Recalcs", "Final Position", "Algorithm Usage");
        println!("{}", "-".repeat(140));

        // Print results for each algorithm
        for result in results {
            let success_str = if result.success { "✓" } else { "✗" };
            let efficiency_str = format!("{:.3}", result.statistics.route_efficiency);
            let final_pos_str = format!("({},{})", result.final_position.x, result.final_position.y);
            
            let usage_str = match &result.algorithm_stats {
                AlgorithmStats::AStar(_) => format!("{} calls", result.timing_data.total_calls()),
                AlgorithmStats::DStarLite(_) => format!("{} calls", result.timing_data.total_calls()),
                AlgorithmStats::Hybrid { a_star_calls, d_star_calls } => {
                    format!("A*:{} D*:{}", a_star_calls, d_star_calls)
                }
            };
            
            let avg_observe_str = format!("{:.2?}", result.timing_data.average_observe_time());
            let avg_find_path_str = format!("{:.2?}", result.timing_data.average_find_path_time());
            let path_recalcs_str = format!("{}", result.timing_data.total_calls());
            
            println!("{:<15} {:<8} {:<8} {:<8} {:<12} {:<15} {:<15} {:<15} {:<15} {:<20}", 
                     result.name,
                     success_str,
                     result.statistics.total_moves,
                     result.statistics.optimal_path_length,
                     efficiency_str,
                     avg_observe_str,
                     avg_find_path_str,
                     path_recalcs_str,
                     final_pos_str,
                     usage_str);
        }

        // Print detailed analysis
        println!();
        println!("=== PERFORMANCE ANALYSIS ===");
        
        let successful_algorithms: Vec<_> = results.iter().filter(|r| r.success).collect();
        
        if !successful_algorithms.is_empty() {
            // Find best performing algorithm by different metrics
            let best_moves = successful_algorithms.iter()
                .min_by_key(|r| r.statistics.total_moves)
                .unwrap();
            
            let best_efficiency = successful_algorithms.iter()
                .min_by(|a, b| a.statistics.route_efficiency.partial_cmp(&b.statistics.route_efficiency).unwrap())
                .unwrap();

            let fewest_recalcs = successful_algorithms.iter()
                .min_by_key(|r| r.timing_data.total_calls())
                .unwrap();

            let fastest_avg_recalc = successful_algorithms.iter()
                .min_by_key(|r| r.timing_data.average_find_path_time())
                .unwrap();

            println!("Best by total moves: {} ({} moves)", best_moves.name, best_moves.statistics.total_moves);
            println!("Best by efficiency: {} ({:.3} efficiency)", best_efficiency.name, best_efficiency.statistics.route_efficiency);
            println!("Fewest path recalculations: {} ({} recalcs)", fewest_recalcs.name, fewest_recalcs.timing_data.total_calls());
            println!("Fastest avg recalculation: {} ({:.2?} avg)", fastest_avg_recalc.name, fastest_avg_recalc.timing_data.average_find_path_time());
            
            // Show path recalculation comparison
            println!();
            println!("=== PATH RECALCULATION ANALYSIS ===");
            for result in successful_algorithms {
                let total_moves = result.statistics.total_moves;
                let recalcs = result.timing_data.total_calls();
                let recalc_ratio = if total_moves > 0 { 
                    recalcs as f64 / total_moves as f64 
                } else { 
                    0.0 
                };
                
                println!("{}: {} recalcs over {} moves ({:.3} recalcs/move)", 
                         result.name, recalcs, total_moves, recalc_ratio);
            }
            
            // Hybrid algorithm breakdown
            println!();
            println!("=== HYBRID ALGORITHM BREAKDOWN ===");
            for result in results {
                if let AlgorithmStats::Hybrid { a_star_calls, d_star_calls } = &result.algorithm_stats {
                    let total_calls = a_star_calls + d_star_calls;
                    if total_calls > 0 {
                        let a_star_pct = (*a_star_calls as f64 / total_calls as f64) * 100.0;
                        let d_star_pct = (*d_star_calls as f64 / total_calls as f64) * 100.0;
                        println!("{}: {} total calls", result.name, total_calls);
                        println!("  • A* usage: {} calls ({:.1}%)", a_star_calls, a_star_pct);
                        println!("  • D* Lite usage: {} calls ({:.1}%)", d_star_calls, d_star_pct);
                        
                        // Performance analysis
                        if *a_star_calls == 1 && *d_star_calls > 0 {
                            println!("  ✓ Optimal hybrid performance: A* used once for initial path, D* Lite handled updates");
                        } else if *a_star_calls > 1 {
                            println!("  ⚠ Multiple A* calls: {} - indicates significant environment changes", a_star_calls);
                        } else if *d_star_calls == 0 {
                            println!("  ⚠ Only A* used - no incremental updates occurred");
                        }
                    }
                }
            }
        } else {
            println!("No algorithms successfully reached the goal.");
        }
    }
}


#[derive(Debug, Clone, Default)]
pub struct TimingData {
    pub observe_times: Vec<Duration>,
    pub find_path_times: Vec<Duration>,
}

    impl TimingData {
        pub fn new() -> Self {
            Self::default()
        }
    
        pub fn average_observe_time(&self) -> Duration {
            if self.observe_times.is_empty() {
                Duration::from_nanos(0)
            } else {
                let total: Duration = self.observe_times.iter().sum();
                total / self.observe_times.len() as u32
            }
        }
        
        pub fn average_find_path_time(&self) -> Duration {
            if self.find_path_times.is_empty() {
                Duration::from_nanos(0)
            } else {
                let total: Duration = self.find_path_times.iter().sum();
                total / self.find_path_times.len() as u32
            }
        }
        
        pub fn total_calls(&self) -> usize {
            self.find_path_times.len()
        }
}

   

