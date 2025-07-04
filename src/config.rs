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

    #[arg(long, default_value = "a_star")]
    pub algorithm: String,

    #[arg(long, default_value_t = 50)]
    pub delay_ms: u64,

    #[arg(long, default_value_t = false)]
    pub no_visualization: bool,
}
