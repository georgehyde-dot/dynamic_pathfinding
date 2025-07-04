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
    
    // Small delay before starting
    std::thread::sleep(std::time::Duration::from_millis(1000));

    let mut simulation = Simulation::new(config);
    let stats = simulation.run();

    println!("\n=== FINAL RESULTS ===");
    println!("{}", stats);
    
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
