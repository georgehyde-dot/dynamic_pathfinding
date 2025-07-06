use clap::Parser;

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = None)]
pub struct Config {
    #[arg(long, default_value_t = 20)]
    pub grid_size: usize,

    #[arg(long, default_value_t = 50)]
    pub num_walls: usize,

    #[arg(long, default_value_t = 10)]
    pub num_obstacles: usize,

    /// Pathfinding algorithm to use
    #[arg(long, default_value = "a_star")]
    #[arg(help = "Algorithm: 'a_star', 'd_star_lite', 'hybrid', or 'all'")]
    pub algorithm: String,

    #[arg(long, default_value_t = 50)]
    pub delay_ms: u64,

    #[arg(long, default_value_t = false)]
    pub no_visualization: bool,

    // New batch simulation parameters
    #[arg(long, default_value_t = false)]
    pub batch_mode: bool,

    #[arg(long, default_value_t = 10)]
    pub num_simulations: usize,

    #[arg(long, default_value_t = 10)]
    pub min_walls: usize,

    #[arg(long, default_value_t = 50)]
    pub max_walls: usize,

    #[arg(long, default_value_t = 5)]
    pub min_obstacles: usize,

    #[arg(long, default_value_t = 15)]
    pub max_obstacles: usize,

    #[arg(long, default_value_t = 300)]
    pub timeout_seconds: u64,

    #[arg(long, default_value = "simulation_results.csv")]
    pub output_file: String,
}
