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
    pub obstacle_timeline: Vec<HashSet<Position>>, // Each entry is obstacles to place at that cycle
    pub obstacle_cycle_interval: usize,
    pub obstacle_persistence_cycles: usize,
}

impl EnvironmentSetup {
    /// Generate a new environment setup with predetermined walls and obstacle timeline
    pub fn generate(config: &Config, seed: Option<u64>) -> Self {
        // Use seed for reproducible results
        let mut rng = if let Some(seed) = seed {
            rand::rngs::StdRng::seed_from_u64(seed)
        } else {
            rand::rngs::StdRng::from_entropy()
        };

        // Generate grid layout
        let start = Position { 
            x: rng.gen_range(0..config.grid_size/2), 
            y: rng.gen_range(0..config.grid_size/2) 
        };
        let goal = Position { 
            x: rng.gen_range(config.grid_size/2..config.grid_size), 
            y: rng.gen_range(config.grid_size/2..config.grid_size) 
        };

        // Generate walls
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

        // Pre-generate obstacle timeline
        let obstacle_cycle_interval = 5;
        let obstacle_persistence_cycles = 5;
        let max_cycles = config.grid_size * config.grid_size; // Estimate max simulation length
        let num_obstacle_cycles = max_cycles / obstacle_cycle_interval;
        
        let mut obstacle_timeline = Vec::new();
        
        for cycle in 0..num_obstacle_cycles {
            let mut obstacle_group = HashSet::new();
            let mut attempts = 0;
            let max_attempts = config.num_obstacles * 10;

            while obstacle_group.len() < config.num_obstacles && attempts < max_attempts {
                let x = rng.gen_range(0..config.grid_size);
                let y = rng.gen_range(0..config.grid_size);
                let pos = Position { x, y };

                // Check if position is valid for obstacle placement
                if pos != start && pos != goal && 
                   !walls.contains(&pos) && 
                   !obstacle_group.contains(&pos) {
                    obstacle_group.insert(pos);
                }
                attempts += 1;
            }
            
            obstacle_timeline.push(obstacle_group);
        }

        println!("Generated environment - Start: {:?}, Goal: {:?}, Walls: {}, Obstacle cycles: {}", 
                 start, goal, walls.len(), obstacle_timeline.len());

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

    /// Create a grid from this environment setup
    pub fn create_grid(&self) -> Grid {
        let mut cells = vec![vec![Cell::Empty; self.grid_size]; self.grid_size];
        
        // Place walls
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
    pub algorithm_stats: AlgorithmStats,
    pub timing_data: TimingData,
    pub success: bool,
    pub final_position: Position,
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
    grid: Grid,
    agent: Agent,
    algorithm: Box<dyn PathfindingAlgorithm>,
    config: Config,
    optimal_path_length: usize,
    environment: EnvironmentSetup,
    active_obstacle_groups: Vec<ObstacleGroup>,
    cycles_since_last_obstacle: usize,
    current_obstacle_cycle: usize,
}

impl Simulation {
    pub fn new(config: Config) -> Self {
        Self::new_with_environment(config, None)
    }

    pub fn new_with_environment(config: Config, environment: Option<EnvironmentSetup>) -> Self {
        let environment = environment.unwrap_or_else(|| EnvironmentSetup::generate(&config, None));
        let grid = environment.create_grid();
        let agent = Agent::new(grid.start);

        let algorithm: Box<dyn PathfindingAlgorithm> = match config.algorithm.as_str() {
            "a_star" => Box::new(AStar::new()),
            "d_star_lite" => Box::new(DStarLite::new(grid.start, grid.goal)),
            "hybrid" => Box::new(HybridAStarDStar::new(grid.start, grid.goal)),
            _ => panic!("Select 'a_star', 'd_star_lite', 'hybrid', or 'all' for algorithm"),
        };

        // Calculate optimal path using A* (no obstacles, only walls)
        let optimal_path_length = Self::calculate_optimal_path_with_astar(&grid);
        
        if optimal_path_length == 0 {
            panic!("No valid path exists from start to goal! Try reducing num_walls.");
        }

        Simulation {
            grid,
            agent,
            algorithm,
            config,
            optimal_path_length,
            environment,
            active_obstacle_groups: Vec::new(),
            cycles_since_last_obstacle: 0,
            current_obstacle_cycle: 0,
        }
    }

    /// Run all algorithms and compare results
    pub fn run_all_algorithms(config: Config) -> Vec<AlgorithmResult> {
        // Generate a random seed for this run, but use it consistently across all algorithms
        let run_seed = rand::random::<u64>();
        let environment = EnvironmentSetup::generate(&config, Some(run_seed));
        
        // Define available algorithms - use names that match the simulation constructor
        let algorithms = [
            AlgorithmRunner::new("a_star", |_start, _goal| Box::new(AStar::new())),
            AlgorithmRunner::new("d_star_lite", |start, goal| Box::new(DStarLite::new(start, goal))),
            AlgorithmRunner::new("hybrid", |start, goal| Box::new(HybridAStarDStar::new(start, goal))),
        ];

        let mut results = Vec::new();

        println!("Running comparison of {} algorithms...", algorithms.len());
        println!("Environment seed: {} (for reproducibility)", run_seed);
        println!("Environment: Grid {}x{}, Walls: {}, Obstacles: {}", 
                 environment.grid_size, environment.grid_size, 
                 environment.walls.len(), config.num_obstacles);
        println!("Start: {:?}, Goal: {:?}", environment.start, environment.goal);
        println!();

        // create grid for both algorithms to use

        let grid = environment.create_grid();

        // Calculate optimal path using A* (no obstacles, only walls)
        let optimal_path_length = Self::calculate_optimal_path_with_astar(&grid);
        
        if optimal_path_length == 0 {
            panic!("No valid path exists from start to goal! Try reducing num_walls.");
        }

        for (i, algorithm_runner) in algorithms.iter().enumerate() {
            println!("Running algorithm {} of {}: {}", i + 1, algorithms.len(), algorithm_runner.name);
            
            // Create a new config for this algorithm run (no visualization)
            let mut algorithm_config = config.clone();
            algorithm_config.no_visualization = true;
            algorithm_config.algorithm = algorithm_runner.name.clone();

            // Create simulation with the shared environment
            let mut simulation = Simulation::new_with_environment_and_algorithm(
                algorithm_config,
                environment.clone(),
                (algorithm_runner.create_algorithm)(environment.start, environment.goal),
                optimal_path_length,
                &grid
            );


            // Run the simulation
            let (statistics, algorithm_stats, timing_data) = simulation.run();
            let success = simulation.agent.position == simulation.grid.goal;
            let final_position = simulation.agent.position;

            results.push(AlgorithmResult {
                name: algorithm_runner.name.clone(),
                statistics,
                algorithm_stats,
                timing_data,
                success,
                final_position,
            });

            println!("Completed: {} - Success: {}, Moves: {}", 
                     algorithm_runner.name, success, results.last().unwrap().statistics.total_moves);
        }

        results
    }

    /// Create simulation with specific environment and algorithm
    pub fn new_with_environment_and_algorithm(
        config: Config, 
        environment: EnvironmentSetup, 
        algorithm: Box<dyn PathfindingAlgorithm>,
        optimal_path_length: usize,
        grid: &Grid
    ) -> Self {

        let agent = Agent::new(grid.start);

        let sim_grid = grid.clone();

        Simulation {
            grid: sim_grid,
            agent,
            algorithm,
            config,
            optimal_path_length,
            environment,
            active_obstacle_groups: Vec::new(),
            cycles_since_last_obstacle: 0,
            current_obstacle_cycle: 0,
        }
    }

    /// Print comparison results in a nice table format
    pub fn print_comparison_results(results: &[AlgorithmResult]) {
        println!("\n=== ALGORITHM COMPARISON RESULTS ===");
        println!();
        
        // Print header
        println!("{:<15} {:<8} {:<8} {:<8} {:<12} {:<15} {:<15} {:<15} {:<15} {:<20}", 
                 "Algorithm", "Success", "Moves", "Optimal", "Efficiency", "Avg Observe", "Avg Find Path", "Total Calls", "Final Position", "Algorithm Usage");
        println!("{}", "-".repeat(110));

        // Print results for each algorithm
        for result in results {
            let success_str = if result.success { "✓" } else { "✗" };
            let efficiency_str = format!("{:.3}", result.statistics.route_efficiency);
            let final_pos_str = format!("({},{})", result.final_position.x, result.final_position.y);
            let extra_moves = result.statistics.total_moves.saturating_sub(result.statistics.optimal_path_length);
            
            let usage_str = match &result.algorithm_stats {
                AlgorithmStats::AStar(calls) => format!("{} calls", calls),
                AlgorithmStats::DStarLite(calls) => format!("{} calls", calls),
                AlgorithmStats::Hybrid { a_star_calls, d_star_calls } => {
                    format!("A*:{} D*:{}", a_star_calls, d_star_calls)
                }
            };
            
            let avg_observe_str = format!("{:.2?}", result.timing_data.average_observe_time());
            let avg_find_path_str = format!("{:.2?}", result.timing_data.average_find_path_time());
            let total_calls_str = format!("{}", result.timing_data.total_calls());
            
            println!("{:<15} {:<8} {:<8} {:<8} {:<12} {:<15} {:<15} {:<15} {:<15} {:<20}", 
                     result.name,
                     success_str,
                     result.statistics.total_moves,
                     result.statistics.optimal_path_length,
                     efficiency_str,
                     avg_observe_str,
                     avg_find_path_str,
                     total_calls_str,
                     final_pos_str,
                     usage_str);
        }

        // Print detailed hybrid algorithm breakdown
        println!();
        println!("=== HYBRID ALGORITHM BREAKDOWN ===");
        let mut found_hybrid = false;
        for result in results {
            if let AlgorithmStats::Hybrid { a_star_calls, d_star_calls } = &result.algorithm_stats {
                found_hybrid = true;
                let total_calls = a_star_calls + d_star_calls;
                if total_calls > 0 {
                    let a_star_pct = (*a_star_calls as f64 / total_calls as f64) * 100.0;
                    let d_star_pct = (*d_star_calls as f64 / total_calls as f64) * 100.0;
                    println!("{}: Total calls: {}", result.name, total_calls);
                    println!("  • A* usage: {} calls ({:.1}%)", a_star_calls, a_star_pct);
                    println!("  • D* Lite usage: {} calls ({:.1}%)", d_star_calls, d_star_pct);
                    println!("  • Average observe time: {:.2?}", result.timing_data.average_observe_time());
                    println!("  • Average find_path time: {:.2?}", result.timing_data.average_find_path_time());
                    
                    // Performance analysis
                    if *a_star_calls == 0 && *d_star_calls > 0 {
                        println!("  ✓ Optimal hybrid performance: Only initial A* setup used, D* Lite handled all runtime updates");
                    } else if *a_star_calls > 0 {
                        println!("  ⚠ Multiple A* calls: {} - indicates significant environment changes", a_star_calls);
                    } else if *d_star_calls == 0 {
                        println!("  ⚠ Only A* used - no incremental updates occurred");
                    }
                    println!();
                }
            }
        }
        
        if !found_hybrid {
            println!("No hybrid algorithms were run in this comparison.");
        }

        println!();

        // Print summary analysis
        let successful_algorithms: Vec<_> = results.iter().filter(|r| r.success).collect();
        
        if !successful_algorithms.is_empty() {
            println!("=== PERFORMANCE ANALYSIS ===");
            
            // Find best performing algorithm
            let best_moves = successful_algorithms.iter()
                .min_by_key(|r| r.statistics.total_moves)
                .unwrap();
            
            let best_efficiency = successful_algorithms.iter()
                .min_by(|a, b| a.statistics.route_efficiency.partial_cmp(&b.statistics.route_efficiency).unwrap())
                .unwrap();

            let fastest_find_path = successful_algorithms.iter()
                .min_by_key(|r| r.timing_data.average_find_path_time())
                .unwrap();

            let fastest_observe = successful_algorithms.iter()
                .min_by_key(|r| r.timing_data.average_observe_time())
                .unwrap();

            println!("Best by moves: {} ({} moves)", best_moves.name, best_moves.statistics.total_moves);
            println!("Best by efficiency: {} ({:.3} efficiency)", best_efficiency.name, best_efficiency.statistics.route_efficiency);
            println!("Fastest find_path: {} ({:.2?} avg)", fastest_find_path.name, fastest_find_path.timing_data.average_find_path_time());
            println!("Fastest observe: {} ({:.2?} avg)", fastest_observe.name, fastest_observe.timing_data.average_observe_time());
            
            // Compare performance
            if successful_algorithms.len() > 1 {
                let move_counts: Vec<_> = successful_algorithms.iter().map(|r| r.statistics.total_moves).collect();
                let min_moves = *move_counts.iter().min().unwrap();
                let max_moves = *move_counts.iter().max().unwrap();
                let move_difference = max_moves - min_moves;
                
                println!("Move count difference: {} moves ({:.1}% variation)", 
                         move_difference, 
                         (move_difference as f64 / min_moves as f64) * 100.0);

                // Timing comparison
                let find_path_times: Vec<_> = successful_algorithms.iter()
                    .map(|r| r.timing_data.average_find_path_time())
                    .collect();
                let min_find_path_time = *find_path_times.iter().min().unwrap();
                let max_find_path_time = *find_path_times.iter().max().unwrap();
                
                println!("Find_path time range: {:.2?} to {:.2?}", min_find_path_time, max_find_path_time);
                
                if max_find_path_time > min_find_path_time {
                    let time_ratio = max_find_path_time.as_nanos() as f64 / min_find_path_time.as_nanos() as f64;
                    println!("Slowest algorithm is {:.1}x slower than fastest", time_ratio);
                }
            }
        } else {
            println!("No algorithms successfully reached the goal.");
        }
    }

    pub fn run(&mut self) -> (Statistics, AlgorithmStats, TimingData) {
        let mut stats = Statistics::new(
            self.config.num_walls, 
            self.config.num_obstacles, 
            self.optimal_path_length
        );

        let mut total_iterations = 0;
        let max_iterations = self.grid.size * self.grid.size * 4;
        
        // Track algorithm calls
        let mut algorithm_calls = 0;
        
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

        while self.agent.position != self.grid.goal && total_iterations < max_iterations {
            // Update obstacle lifecycle using pre-generated timeline
            self.update_obstacles_from_timeline();
            
            // Time the observe call
            let observe_start = Instant::now();
            self.agent.observe(&self.grid);
            let observe_duration = observe_start.elapsed();
            timing_data.observe_times.push(observe_duration);
            
            // Time the find_path call
            let find_path_start = Instant::now();
            let path = self.algorithm.find_path(
                &self.grid,
                self.agent.position,
                self.grid.goal,
                &self.agent.known_obstacles,
            );
            let find_path_duration = find_path_start.elapsed();
            timing_data.find_path_times.push(find_path_duration);
            
            // Track algorithm calls
            algorithm_calls += 1;

            if let Some(path) = path {
                // Reset stuck counter when path is found
                stuck_attempts = 0;
                
                if path.len() > 1 {
                    let next_pos = path[1];
                    self.agent.move_to(next_pos);
                    stats.total_moves += 1;
                    
                    // Only print and sleep if visualization is enabled
                    if !self.config.no_visualization {
                        self.clear_screen();
                        println!("=== PATHFINDING SIMULATION ===");
                        println!("Algorithm: {} | Step: {} | Moves: {} | Active obstacle groups: {}", 
                                 self.config.algorithm, total_iterations + 1, stats.total_moves, self.active_obstacle_groups.len());
                        println!("Agent position: ({}, {})", self.agent.position.x, self.agent.position.y);
                        println!("Goal position: ({}, {})", self.grid.goal.x, self.grid.goal.y);
                        println!("Original optimal path (A*): {}", self.optimal_path_length);
                        println!("Obstacle cycle: {} | Cycles until next: {}", 
                                 self.current_obstacle_cycle,
                                 self.environment.obstacle_cycle_interval - self.cycles_since_last_obstacle);
                        
                        // Show timing info if enabled
                        println!("Last observe: {:.2?} | Last find_path: {:.2?}", 
                                 observe_duration, find_path_duration);
                        
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
                stuck_attempts += 1;
                
                if stuck_attempts <= MAX_STUCK_ATTEMPTS {
                    // Wait and try again next turn
                    stats.total_moves += 1; // Count waiting as a move
                    
                    if !self.config.no_visualization {
                        self.clear_screen();
                        println!("=== PATHFINDING SIMULATION ===");
                        println!("Agent stuck at position {:?} - waiting... (attempt {}/{})", 
                                 self.agent.position, stuck_attempts, MAX_STUCK_ATTEMPTS);
                        println!("Algorithm: {} | Step: {} | Moves: {} | Active obstacle groups: {}", 
                                 self.config.algorithm, total_iterations + 1, stats.total_moves, self.active_obstacle_groups.len());
                        println!("Last observe: {:.2?} | Last find_path: {:.2?}", 
                                 observe_duration, find_path_duration);
                        self.grid.print_grid(Some(self.agent.position));
                        thread::sleep(Duration::from_millis(self.config.delay_ms));
                    } else {
                        println!("Agent stuck at position {:?} - waiting... (attempt {}/{})", 
                                 self.agent.position, stuck_attempts, MAX_STUCK_ATTEMPTS);
                    }
                } else {
                    // Truly stuck after max attempts
                    if !self.config.no_visualization {
                        self.clear_screen();
                        println!("=== PATHFINDING SIMULATION ===");
                        println!("FAILURE: Agent permanently stuck at position {:?} after {} attempts", 
                                 self.agent.position, MAX_STUCK_ATTEMPTS);
                        self.grid.print_grid(Some(self.agent.position));
                    } else {
                        println!("Agent permanently stuck at position {:?} after {} attempts", 
                                 self.agent.position, MAX_STUCK_ATTEMPTS);
                    }
                    break;
                }
            }
            
            total_iterations += 1;
            if total_iterations >= max_iterations {
                panic!("Reached max iterations, exiting");
            }
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
            println!("Algorithm: {}", self.config.algorithm);
            println!("Final position: ({}, {})", self.agent.position.x, self.agent.position.y);
            println!("Total steps: {} | Total moves: {}", total_iterations, stats.total_moves);
            println!("Original optimal path (A*): {}", self.optimal_path_length);
            
            // Show timing summary
            println!("Average observe time: {:.2?}", timing_data.average_observe_time());
            println!("Average find_path time: {:.2?}", timing_data.average_find_path_time());
            
            // Calculate final optimal path
            let final_optimal_length = Self::calculate_optimal_path_with_astar(&self.grid);
            println!("Final optimal path (A*): {}", final_optimal_length);
            
            self.grid.print_grid(Some(self.agent.position));
        }

        stats.calculate_efficiency();
        
        // Create appropriate algorithm stats based on algorithm type
        let algorithm_stats = match self.config.algorithm.as_str() {
            "a_star" => AlgorithmStats::AStar(algorithm_calls),
            "d_star_lite" => AlgorithmStats::DStarLite(algorithm_calls),
            "hybrid" => {
                let (a_star_calls, d_star_calls) = self.algorithm.get_usage_stats();
                AlgorithmStats::Hybrid { a_star_calls, d_star_calls }
            },
            _ => AlgorithmStats::AStar(algorithm_calls), // fallback
        };

        (stats, algorithm_stats, timing_data)
    }

    /// Update obstacles using the pre-generated timeline
    fn update_obstacles_from_timeline(&mut self) {
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

        // Place new obstacles if it's time and we have more in the timeline
        if self.cycles_since_last_obstacle >= self.environment.obstacle_cycle_interval {
            if self.current_obstacle_cycle < self.environment.obstacle_timeline.len() {
                self.place_obstacle_group_from_timeline();
                self.current_obstacle_cycle += 1;
            }
            self.cycles_since_last_obstacle = 0;
        }
    }

    /// Place obstacles from the pre-generated timeline
    fn place_obstacle_group_from_timeline(&mut self) {
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
}

#[derive(Debug, Clone)]
pub struct TimingData {
    pub observe_times: Vec<Duration>,
    pub find_path_times: Vec<Duration>,
}

impl TimingData {
    pub fn new() -> Self {
        TimingData {
            observe_times: Vec::new(),
            find_path_times: Vec::new(),
        }
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
        // Should be the same for both, but use find_path as it's the main operation
        self.find_path_times.len()
    }
}
