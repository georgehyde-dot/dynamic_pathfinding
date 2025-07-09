use clap::Parser;

use dynamic_pathfinding::batch_simulation::BatchSimulation;
use dynamic_pathfinding::config::Config;
use dynamic_pathfinding::simulation::Simulation;
use std::time::Duration;

fn main() {
    let config = Config::parse();

    println!("Starting pathfinding simulation...");
    println!("Grid size: {}x{}", config.grid_size, config.grid_size);
    println!(
        "Walls: {}, Obstacles: {}",
        config.num_walls, config.num_obstacles
    );
    println!("Algorithm: {}", config.algorithm);

    if config.no_visualization || config.batch_mode {
        println!("Visualization disabled - running in fast mode");
    } else {
        println!("Visualization enabled with {}ms delay", config.delay_ms);
        println!("Press Ctrl+C to stop the simulation");
    }

    if config.quiet {
        println!("Quiet mode enabled - minimal output");
    }

    println!();

    // Small delay before starting (only if visualization is enabled)
    if !config.no_visualization || !config.batch_mode {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }

    // Check if we should run batch mode
    if config.batch_mode {
        let mut batch_sim = BatchSimulation::new(config.clone());
        match batch_sim.run() {
            Ok(()) => {
                if !config.quiet {
                    batch_sim.print_summary();
                }
            }
            Err(e) => {
                eprintln!("Batch simulation failed: {}", e);
                std::process::exit(1);
            }
        }
    } else if config.algorithm == "all" {
        // Run all algorithms and compare results
        match Simulation::run_all_algorithms(config) {
            Ok(results) => {
                Simulation::print_comparison_results(&results);
            }
            Err(e) => {
                eprintln!("Error running all algorithms: {}", e);
                std::process::exit(1);
            }
        }
    } else {
        // Run single algorithm
        if let Ok(mut simulation) = Simulation::new(config.clone()) {
            let (stats, algorithm_stats, timing_data) = simulation.run();

            println!("\n=== FINAL RESULTS ===");
            println!("{}", stats);
            println!("{}", algorithm_stats);

            // Print timing information
            println!("\n=== TIMING ANALYSIS ===");
            println!("Total pathfinding calls: {}", timing_data.total_calls());
            println!(
                "Average find_path time: {:.2?}",
                timing_data.average_find_path_time()
            );

            if timing_data.total_calls() > 0 {
                let total_find_path_time: Duration = timing_data.find_path_times.iter().sum();

                println!("Total time in find_path: {:.2?}", total_find_path_time);

                // Additional analysis
                if stats.total_moves > 0 {
                    let extra_moves = stats.total_moves.saturating_sub(stats.optimal_path_length);
                    println!(
                        "Extra moves due to obstacles/limited vision: {}",
                        extra_moves
                    );
                }
            } else {
                println!("Failed to create simulation - likely no valid path exists with current configuration");
                println!("Try reducing --num-walls or increasing --grid-size");
            }
        }
    }
}
