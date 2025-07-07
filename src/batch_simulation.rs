use crate::config::Config;
use crate::simulation::{Simulation, AlgorithmResult};
use crate::statistics::{ AlgorithmStats};
use std::fs::OpenOptions;
use std::io::Write;
use std::time::{Duration, Instant};
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct BatchResult {
    pub simulation_id: usize,
    pub algorithm: String,
    pub grid_size: usize,
    pub num_walls: usize,
    pub num_obstacles: usize,
    pub success: bool,
    pub total_moves: usize,
    pub optimal_path_length: usize,
    pub route_efficiency: f64,
    pub execution_time_ms: u64,
    pub a_star_calls: usize,
    pub d_star_calls: usize,
    pub average_observe_time_ns: u64,
    pub average_find_path_time_ns: u64,
    pub total_pathfinding_calls: usize,
}

pub struct BatchSimulation {
    config: Config,
    results: Vec<BatchResult>,
    start_time: Instant,
    batch_size: usize,           // Add this
    total_results_written: usize, // Add this
}

impl BatchSimulation {
    pub fn new(config: Config) -> Self {
        BatchSimulation {
            config,
            results: Vec::new(),
            start_time: Instant::now(),
            batch_size: 100,             // Add this
            total_results_written: 0,    // Add this
        }
    }

    pub fn run(&mut self) -> Result<(), String> {
        if self.initialize_csv_file().is_ok() {
            println!("Initialized CSV");
        }
        if !self.config.quiet {
            println!("=== BATCH SIMULATION STARTED ===");
            println!("Grid size: {}", self.config.grid_size);
            println!("Walls range: {} to {}", self.config.min_walls, self.config.max_walls);
            println!("Obstacles range: {} to {}", self.config.min_obstacles, self.config.max_obstacles);
            println!("Simulations per configuration: {}", self.config.num_simulations);
            println!("Timeout: {} seconds", self.config.timeout_seconds);
            println!("Algorithm: {}", self.config.algorithm);
            println!("Output file: {}", self.config.output_file);
            println!();
        }

        let total_configurations = self.count_total_configurations();
        let total_simulations = total_configurations * self.config.num_simulations;
        
        if !self.config.quiet {
            println!("Total configurations to test: {}", total_configurations);
            println!("Total simulations to run: {}", total_simulations);
            println!();
        }

        let mut configuration_count = 0;
        let mut completed_simulations = 0;
        let timeout_duration = Duration::from_secs(self.config.timeout_seconds);

        // Progress reporting variables
        let mut last_progress_report = Instant::now();
        let progress_interval = Duration::from_secs(10); // Report every 10 seconds

        // Iterate through all combinations of walls and obstacles
        for num_walls in self.config.min_walls..=self.config.max_walls {
            for num_obstacles in self.config.min_obstacles..=self.config.max_obstacles {
                configuration_count += 1;
                
                // Check timeout
                if self.start_time.elapsed() > timeout_duration {
                    if !self.config.quiet {
                        println!("⏰ Timeout reached after {} configurations", configuration_count - 1);
                    }
                    break;
                }

                if !self.config.quiet {
                    println!("Configuration {}/{}: {} walls, {} obstacles", 
                             configuration_count, total_configurations, num_walls, num_obstacles);
                }

                // Run simulations for this configuration
                let sims_completed = self.run_configuration(num_walls, num_obstacles)?;
                completed_simulations += sims_completed;

                if self.results.len() >= self.batch_size {
                    self.flush_results_to_csv()?;
                }

                // Progress reporting - show progress every 10 seconds regardless of quiet mode
                if last_progress_report.elapsed() > progress_interval {
                    let progress_percentage = (completed_simulations as f64 / total_simulations as f64) * 100.0;
                    let elapsed = self.start_time.elapsed();
                    let estimated_total = if completed_simulations > 0 {
                        elapsed.mul_f64(total_simulations as f64 / completed_simulations as f64)
                    } else {
                        Duration::from_secs(0)
                    };
                    let remaining = estimated_total.saturating_sub(elapsed);
                    
                    println!("Progress: {:.1}% ({}/{}) - Elapsed: {:.1}s - ETA: {:.1}s - Batches written: {}", 
                             progress_percentage, completed_simulations, total_simulations,
                             elapsed.as_secs_f64(), remaining.as_secs_f64(), 
                             self.total_results_written / self.batch_size);
                    last_progress_report = Instant::now();
                }
            }
            
            // Check timeout again at outer loop level
            if self.start_time.elapsed() > timeout_duration {
                break;
            }
        }

        if !self.results.is_empty() {
            self.flush_results_to_csv()?;
        }

        if !self.config.quiet {
            println!("\n=== BATCH SIMULATION COMPLETED ===");
            println!("Total results collected: {}", self.results.len());
            println!("Results saved to: {}", self.config.output_file);
            println!("Total time: {:.2?}", self.start_time.elapsed());
        } else {
            println!("Batch simulation completed: {} results in {:.1}s -> {}", 
                     self.results.len(), self.start_time.elapsed().as_secs_f64(), self.config.output_file);
        }

        Ok(())
    }

    fn count_total_configurations(&self) -> usize {
        let wall_count = (self.config.max_walls - self.config.min_walls) + 1;
        let obstacle_count = (self.config.max_obstacles - self.config.min_obstacles) + 1;
        wall_count * obstacle_count
    }

    fn run_configuration(&mut self, num_walls: usize, num_obstacles: usize) -> Result<usize, String> {
        // Create a configuration for this specific run
        let mut run_config = self.config.clone();
        run_config.num_walls = num_walls;
        run_config.num_obstacles = num_obstacles;
        run_config.no_visualization = true; // Always disable visualization in batch mode
        run_config.quiet = true; // Force quiet mode for individual simulations

        let mut completed_count = 0;

        for sim_id in 0..self.config.num_simulations {
            // Check timeout before each simulation
            let timeout_duration = Duration::from_secs(self.config.timeout_seconds);
            if self.start_time.elapsed() > timeout_duration {
                return Ok(completed_count);
            }

            let simulation_start = Instant::now();
            
            if self.config.algorithm == "all" {
                // Run all algorithms for this configuration
                match Simulation::run_all_algorithms(run_config.clone()) {
                    Ok(results) => {
                        for algorithm_result in results {
                            let batch_result = self.convert_algorithm_result_to_batch_result(
                                algorithm_result,
                                sim_id,
                                num_walls,
                                num_obstacles,
                                simulation_start.elapsed()
                            );
                            self.results.push(batch_result);
                        }
                    }
                    Err(_e) => {
                        let algorithms = ["a_star", "d_star_lite"];
                        for algorithm in &algorithms {
                            let failed_result = BatchResult {
                                simulation_id: sim_id,
                                algorithm: algorithm.to_string(),
                                grid_size: self.config.grid_size,
                                num_walls,
                                num_obstacles,
                                success: false,
                                total_moves: 0,
                                optimal_path_length: 0,
                                route_efficiency: 0.0,
                                execution_time_ms: simulation_start.elapsed().as_millis() as u64,
                                a_star_calls: 0,
                                d_star_calls: 0,
                                average_observe_time_ns: 0,
                                average_find_path_time_ns: 0,
                                total_pathfinding_calls: 0,
                            };
                            self.results.push(failed_result);
                        }
                    }
                }
            } else {
                // Run single algorithm with error handling
                match Simulation::new(run_config.clone()) {
                    Ok(mut simulation) => {
                        let (stats, algorithm_stats, timing_data) = simulation.run();
                        
                        let batch_result = BatchResult {
                            simulation_id: sim_id,
                            algorithm: self.config.algorithm.clone(),
                            grid_size: self.config.grid_size,
                            num_walls,
                            num_obstacles,
                            success: simulation.agent.position == simulation.grid.goal,
                            total_moves: stats.total_moves,
                            optimal_path_length: stats.optimal_path_length,
                            route_efficiency: stats.route_efficiency,
                            execution_time_ms: simulation_start.elapsed().as_millis() as u64,
                            a_star_calls: match algorithm_stats {
                                AlgorithmStats::AStar(calls) => calls,
                                AlgorithmStats::Hybrid { a_star_calls, .. } => a_star_calls,
                                _ => 0,
                            },
                            d_star_calls: match algorithm_stats {
                                AlgorithmStats::DStarLite(calls) => calls,
                                AlgorithmStats::Hybrid { d_star_calls, .. } => d_star_calls,
                                _ => 0,
                            },
                            average_observe_time_ns: timing_data.average_observe_time().as_nanos() as u64,
                            average_find_path_time_ns: timing_data.average_find_path_time().as_nanos() as u64,
                            total_pathfinding_calls: timing_data.total_calls(),
                        };
                        
                        self.results.push(batch_result);
                    }
                    Err(_e) => {
                        let failed_result = BatchResult {
                            simulation_id: sim_id,
                            algorithm: self.config.algorithm.clone(),
                            grid_size: self.config.grid_size,
                            num_walls,
                            num_obstacles,
                            success: false,
                            total_moves: 0,
                            optimal_path_length: 0,
                            route_efficiency: 0.0,
                            execution_time_ms: simulation_start.elapsed().as_millis() as u64,
                            a_star_calls: 0,
                            d_star_calls: 0,
                            average_observe_time_ns: 0,
                            average_find_path_time_ns: 0,
                            total_pathfinding_calls: 0,
                        };
                        
                        self.results.push(failed_result);
                    }
                }
            }
            
            completed_count += 1;
        }
        if self.results.len() >= self.batch_size {
            self.flush_results_to_csv()?;
        }
        Ok(completed_count)
    }

    fn convert_algorithm_result_to_batch_result(
        &self,
        result: AlgorithmResult,
        sim_id: usize,
        num_walls: usize,
        num_obstacles: usize,
        execution_time: Duration,
    ) -> BatchResult {
        BatchResult {
            simulation_id: sim_id,
            algorithm: result.name,
            grid_size: self.config.grid_size,
            num_walls,
            num_obstacles,
            success: result.success,
            total_moves: result.statistics.total_moves,
            optimal_path_length: result.statistics.optimal_path_length,
            route_efficiency: result.statistics.route_efficiency,
            execution_time_ms: execution_time.as_millis() as u64,
            a_star_calls: match result.algorithm_stats {
                AlgorithmStats::AStar(calls) => calls,
                AlgorithmStats::Hybrid { a_star_calls, .. } => a_star_calls,
                _ => 0,
            },
            d_star_calls: match result.algorithm_stats {
                AlgorithmStats::DStarLite(calls) => calls,
                AlgorithmStats::Hybrid { d_star_calls, .. } => d_star_calls,
                _ => 0,
            },
            average_observe_time_ns: result.timing_data.average_observe_time().as_nanos() as u64,
            average_find_path_time_ns: result.timing_data.average_find_path_time().as_nanos() as u64,
            total_pathfinding_calls: result.timing_data.total_calls(),
        }
    }

    pub fn with_batch_size(mut self, batch_size: usize) -> Self {
        self.batch_size = batch_size;
        self
    }

    fn flush_results_to_csv(&mut self) -> Result<(), String> {
        if self.results.is_empty() {
            return Ok(());
        }

        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.config.output_file)
            .map_err(|e| format!("Failed to open output file for appending: {}", e))?;

        for result in &self.results {
            writeln!(file, "{},{},{},{},{},{},{},{},{:.6},{},{},{},{},{},{}",
                result.simulation_id, result.algorithm, result.grid_size, result.num_walls, result.num_obstacles,
                result.success, result.total_moves, result.optimal_path_length, result.route_efficiency,
                result.execution_time_ms, result.a_star_calls, result.d_star_calls, result.average_observe_time_ns,
                result.average_find_path_time_ns, result.total_pathfinding_calls
            ).map_err(|e| format!("Failed to write data row: {}", e))?;
        }

        self.total_results_written += self.results.len();
        if !self.config.quiet {
            println!("Flushed {} results to CSV (total: {})", self.results.len(), self.total_results_written);
        }
        self.results.clear();
        Ok(())
    }

    fn initialize_csv_file(&self) -> Result<(), String> {
        let mut file = std::fs::File::create(&self.config.output_file)
            .map_err(|e| format!("Failed to create output file: {}", e))?;

        writeln!(file, "simulation_id,algorithm,grid_size,num_walls,num_obstacles,success,total_moves,optimal_path_length,route_efficiency,execution_time_ms,a_star_calls,d_star_calls,average_observe_time_ns,average_find_path_time_ns,total_pathfinding_calls")
            .map_err(|e| format!("Failed to write header: {}", e))?;

        if !self.config.quiet {
            println!("Initialized CSV file: {}", self.config.output_file);
        }
        Ok(())
    }

    pub fn print_summary(&self) {
        if self.results.is_empty() {
            println!("No results to summarize.");
            return;
        }

        println!("\n=== BATCH SIMULATION SUMMARY ===");
        
        // Group results by algorithm
        let mut algorithm_groups: HashMap<String, Vec<&BatchResult>> = HashMap::new();
        for result in &self.results {
            algorithm_groups.entry(result.algorithm.clone())
                .or_default()
                .push(result);
        }

        for (algorithm, results) in algorithm_groups {
            println!("\n{} Algorithm Results:", algorithm);
            let successful = results.iter().filter(|r| r.success).count();
            let total = results.len();
            let success_rate = (successful as f64 / total as f64) * 100.0;
            
            println!("  Success rate: {}/{} ({:.1}%)", successful, total, success_rate);
            
            if successful > 0 {
                let successful_results: Vec<_> = results.iter().filter(|r| r.success).collect();
                let avg_moves: f64 = successful_results.iter().map(|r| r.total_moves as f64).sum::<f64>() / successful_results.len() as f64;
                let avg_efficiency: f64 = successful_results.iter().map(|r| r.route_efficiency).sum::<f64>() / successful_results.len() as f64;
                let avg_time: f64 = successful_results.iter().map(|r| r.execution_time_ms as f64).sum::<f64>() / successful_results.len() as f64;
                
                println!("  Average moves: {:.1}", avg_moves);
                println!("  Average efficiency: {:.3}", avg_efficiency);
                println!("  Average execution time: {:.1}ms", avg_time);
            }
        }
    }
}