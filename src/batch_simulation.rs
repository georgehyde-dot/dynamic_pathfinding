use crate::config::Config;
use crate::simulation::{Simulation, EnvironmentSetup, AlgorithmResult};
use crate::statistics::{Statistics, AlgorithmStats};
use std::fs::File;
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
}

impl BatchSimulation {
    pub fn new(config: Config) -> Self {
        BatchSimulation {
            config,
            results: Vec::new(),
            start_time: Instant::now(),
        }
    }

    pub fn run(&mut self) -> Result<(), String> {
        println!("=== BATCH SIMULATION STARTED ===");
        println!("Grid size: {}", self.config.grid_size);
        println!("Walls range: {} to {}", self.config.min_walls, self.config.max_walls);
        println!("Obstacles range: {} to {}", self.config.min_obstacles, self.config.max_obstacles);
        println!("Simulations per configuration: {}", self.config.num_simulations);
        println!("Timeout: {} seconds", self.config.timeout_seconds);
        println!("Algorithm: {}", self.config.algorithm);
        println!("Output file: {}", self.config.output_file);
        println!();

        let total_configurations = self.count_total_configurations();
        println!("Total configurations to test: {}", total_configurations);
        println!("Total simulations to run: {}", total_configurations * self.config.num_simulations);
        println!();

        let mut configuration_count = 0;
        let timeout_duration = Duration::from_secs(self.config.timeout_seconds);

        // Iterate through all combinations of walls and obstacles
        for num_walls in self.config.min_walls..=self.config.max_walls {
            for num_obstacles in self.config.min_obstacles..=self.config.max_obstacles {
                configuration_count += 1;
                
                // Check timeout
                if self.start_time.elapsed() > timeout_duration {
                    println!("⏰ Timeout reached after {} configurations", configuration_count - 1);
                    break;
                }

                println!("Configuration {}/{}: {} walls, {} obstacles", 
                         configuration_count, total_configurations, num_walls, num_obstacles);

                // Run simulations for this configuration
                self.run_configuration(num_walls, num_obstacles)?;
            }
            
            // Check timeout again at outer loop level
            if self.start_time.elapsed() > timeout_duration {
                break;
            }
        }

        // Export results to CSV
        self.export_to_csv()?;

        println!("\n=== BATCH SIMULATION COMPLETED ===");
        println!("Total results collected: {}", self.results.len());
        println!("Results saved to: {}", self.config.output_file);
        println!("Total time: {:.2?}", self.start_time.elapsed());

        Ok(())
    }

    fn count_total_configurations(&self) -> usize {
        let wall_count = (self.config.max_walls - self.config.min_walls) + 1;
        let obstacle_count = (self.config.max_obstacles - self.config.min_obstacles) + 1;
        wall_count * obstacle_count
    }

    fn run_configuration(&mut self, num_walls: usize, num_obstacles: usize) -> Result<(), String> {
        // Create a configuration for this specific run
        let mut run_config = self.config.clone();
        run_config.num_walls = num_walls;
        run_config.num_obstacles = num_obstacles;
        run_config.no_visualization = true; // Always disable visualization in batch mode

        for sim_id in 0..self.config.num_simulations {
            // Check timeout before each simulation
            let timeout_duration = Duration::from_secs(self.config.timeout_seconds);
            if self.start_time.elapsed() > timeout_duration {
                println!("  ⏰ Timeout reached during simulation {}", sim_id);
                return Ok(());
            }

            print!("  Simulation {}/{}: ", sim_id + 1, self.config.num_simulations);
            
            let simulation_start = Instant::now();
            
            if self.config.algorithm == "all" {
                // Run all algorithms for this configuration
                let results = Simulation::run_all_algorithms(run_config.clone());
                
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
            } else {
                // Run single algorithm - make sure the algorithm is set correctly
                let mut simulation = Simulation::new(run_config.clone());
                let (stats, algorithm_stats, timing_data) = simulation.run();
                
                let batch_result = BatchResult {
                    simulation_id: sim_id,
                    algorithm: self.config.algorithm.clone(), // This should be the correct algorithm
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
            
            print!("✓ ");
        }
        println!("Done");
        Ok(())
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

    fn export_to_csv(&self) -> Result<(), String> {
        let mut file = File::create(&self.config.output_file)
            .map_err(|e| format!("Failed to create output file: {}", e))?;

        // Write CSV header
        writeln!(file, "simulation_id,algorithm,grid_size,num_walls,num_obstacles,success,total_moves,optimal_path_length,route_efficiency,execution_time_ms,a_star_calls,d_star_calls,average_observe_time_ns,average_find_path_time_ns,total_pathfinding_calls")
            .map_err(|e| format!("Failed to write header: {}", e))?;

        // Write data rows
        for result in &self.results {
            writeln!(file, "{},{},{},{},{},{},{},{},{:.6},{},{},{},{},{},{}",
                result.simulation_id,
                result.algorithm,
                result.grid_size,
                result.num_walls,
                result.num_obstacles,
                result.success,
                result.total_moves,
                result.optimal_path_length,
                result.route_efficiency,
                result.execution_time_ms,
                result.a_star_calls,
                result.d_star_calls,
                result.average_observe_time_ns,
                result.average_find_path_time_ns,
                result.total_pathfinding_calls
            ).map_err(|e| format!("Failed to write data row: {}", e))?;
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
                .or_insert_with(Vec::new)
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