use clap::Parser;

use dynamic_pathfinding::config::Config;
use dynamic_pathfinding::simulation::Simulation;

fn main() {
    let config = Config::parse();

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
        let stats = simulation.run();

        println!("\n=== FINAL RESULTS ===");
        println!("{}", stats);
        
        // Debug information for D* Lite
        if config.algorithm == "d_star_lite" && stats.total_moves == 0 {
            println!("\nDEBUG: D* Lite returned 0 moves - this indicates the algorithm failed to find a path");
            println!("This could be due to:");
            println!("1. Algorithm initialization issues");
            println!("2. Incorrect key calculations");
            println!("3. Path extraction problems");
            println!("Try running with A* to verify the grid has a valid path:");
            println!("cargo run -- --algorithm a_star --grid-size {} --num-walls {}", 
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
