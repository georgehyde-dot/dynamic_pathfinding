use clap::Parser;

use dynamic_pathfinding::config::Config;
use dynamic_pathfinding::simulation::Simulation;
use dynamic_pathfinding::batch_simulation::BatchSimulation;
use std::time::Duration;

fn main() {
    let config = Config::parse();

    // Check if we're running in batch mode
    if config.batch_mode {
        run_batch_simulation(config);
    } else {
        run_single_simulation(config);
    }
}

fn run_batch_simulation(config: Config) {
    println!("=== BATCH SIMULATION MODE ===");
    println!("Algorithm specified: {}", config.algorithm); // Add this debug line
    
    // Validate batch parameters
    if config.min_walls > config.max_walls {
        eprintln!("Error: min_walls ({}) cannot be greater than max_walls ({})", 
                  config.min_walls, config.max_walls);
        return;
    }
    
    if config.min_obstacles > config.max_obstacles {
        eprintln!("Error: min_obstacles ({}) cannot be greater than max_obstacles ({})", 
                  config.min_obstacles, config.max_obstacles);
        return;
    }

    if config.num_simulations == 0 {
        eprintln!("Error: num_simulations must be greater than 0");
        return;
    }

    // Validate algorithm
    let valid_algorithms = ["a_star", "d_star_lite", "hybrid", "all"];
    if !valid_algorithms.contains(&config.algorithm.as_str()) {
        eprintln!("Error: Invalid algorithm '{}'. Valid options are: {:?}", 
                  config.algorithm, valid_algorithms);
        return;
    }

    // Create and run batch simulation
    let mut batch_sim = BatchSimulation::new(config);
    
    match batch_sim.run() {
        Ok(()) => {
            batch_sim.print_summary();
            println!("\n✅ Batch simulation completed successfully!");
        }
        Err(e) => {
            eprintln!("❌ Batch simulation failed: {}", e);
        }
    }
}

fn run_single_simulation(config: Config) {
    println!("Starting pathfinding simulation...");
    println!("Grid size: {}x{}", config.grid_size, config.grid_size);
    println!("Walls: {}, Obstacles: {}", config.num_walls, config.num_obstacles);
    println!("Algorithm: {}", config.algorithm);
    
    if config.no_visualization {
        println!("Visualization disabled - running in fast mode");
    } else {
        println!("Visualization enabled with {}ms delay", config.delay_ms);
        println!("Press Ctrl+C to stop the simulation");
    }
    
    println!();
    
    // Small delay before starting (only if visualization is enabled)
    if !config.no_visualization {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }

    // Check if we should run all algorithms or just one
    if config.algorithm == "all" {
        // Run all algorithms and compare results
        let results = Simulation::run_all_algorithms(config);
        Simulation::print_comparison_results(&results);
    } else {
        // Run single algorithm
        let mut simulation = Simulation::new(config.clone());
        let (stats, algorithm_stats, timing_data) = simulation.run();

        println!("\n=== FINAL RESULTS ===");
        println!("{}", stats);
        println!("{}", algorithm_stats);
        
        // Print timing information
        println!("\n=== TIMING ANALYSIS ===");
        println!("Total pathfinding calls: {}", timing_data.total_calls());
        println!("Average observe time: {:.2?}", timing_data.average_observe_time());
        println!("Average find_path time: {:.2?}", timing_data.average_find_path_time());
        
        if timing_data.total_calls() > 0 {
            let total_observe_time: Duration = timing_data.observe_times.iter().sum();
            let total_find_path_time: Duration = timing_data.find_path_times.iter().sum();
            let total_algorithm_time = total_observe_time + total_find_path_time;
            
            println!("Total time in observe: {:.2?}", total_observe_time);
            println!("Total time in find_path: {:.2?}", total_find_path_time);
            println!("Total algorithm time: {:.2?}", total_algorithm_time);
            
            if total_algorithm_time.as_nanos() > 0 {
                let observe_percentage = (total_observe_time.as_nanos() as f64 / total_algorithm_time.as_nanos() as f64) * 100.0;
                let find_path_percentage = (total_find_path_time.as_nanos() as f64 / total_algorithm_time.as_nanos() as f64) * 100.0;
                
                println!("Time breakdown: {:.1}% observe, {:.1}% find_path", observe_percentage, find_path_percentage);
            }
        }
        
        // Debug information for D* Lite
        if config.algorithm == "d_star_lite" && stats.total_moves == 0 {
            println!("\nDEBUG: D* Lite returned 0 moves - this indicates the algorithm failed to find a path");
            println!("This could be due to:");
            println!("1. Algorithm initialization issues");
            println!("2. Incorrect key calculations");
            println!("3. Path extraction problems");
            println!("Try running with A* or hybrid to verify the grid has a valid path:");
            println!("cargo run -- --algorithm a_star --grid-size {} --num-walls {}", 
                     config.grid_size, config.num_walls);
            println!("cargo run -- --algorithm hybrid --grid-size {} --num-walls {}", 
                     config.grid_size, config.num_walls);
        }
        
        // Additional analysis
        if stats.total_moves > 0 {
            let extra_moves = stats.total_moves.saturating_sub(stats.optimal_path_length);
            println!("Extra moves due to obstacles/limited vision: {}", extra_moves);
            
            if stats.route_efficiency < 0.5 {
                println!("Note: Very low efficiency - agent took significantly more moves than optimal");
            } else if stats.route_efficiency > 0.9 {
                println!("Note: High efficiency - agent found near-optimal path despite obstacles");
            }
        }
    }
}
