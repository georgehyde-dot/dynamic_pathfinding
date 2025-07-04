use std::fmt;

pub struct Statistics {
    pub total_moves: usize,
    pub num_obstacles: usize,
    pub num_walls: usize,
    pub route_efficiency: f64,
    pub optimal_path_length: usize,
}

impl Statistics {
    pub fn new(num_walls: usize, num_obstacles: usize, optimal_path_length: usize) -> Self {
        Statistics {
            total_moves: 0,
            num_obstacles,
            num_walls,
            route_efficiency: 0.0,
            optimal_path_length,
        }
    }

    pub fn calculate_efficiency(&mut self) {
        if self.total_moves > 0 && self.optimal_path_length > 0 {
            // Efficiency = optimal_length / actual_moves
            // Values closer to 1.0 are better, values < 1.0 mean we did better than optimal (shouldn't happen)
            // Values > 1.0 mean we took more moves than optimal
            self.route_efficiency = self.total_moves as f64 / self.optimal_path_length as f64 ;
        } else {
            self.route_efficiency = 0.0;
        }
    }
}

impl fmt::Display for Statistics {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Total Moves: {}", self.total_moves)?;
        writeln!(f, "Optimal Path Length: {}", self.optimal_path_length)?;
        writeln!(f, "Number of Walls: {}", self.num_walls)?;
        writeln!(f, "Number of Obstacles: {}", self.num_obstacles)?;
        writeln!(f, "Route Efficiency: {:.3}", self.route_efficiency)?;
        
        if self.route_efficiency > 0.0 {
            let efficiency_percentage = (self.route_efficiency * 100.0).min(100.0);
            writeln!(f, "Efficiency Percentage: {:.1}%", efficiency_percentage)?;
            
            if self.route_efficiency < 1.0 {
                writeln!(f, "Note: Efficiency > 100% (took fewer moves than pre-obstacle optimal)")?;
            }
        }
        
        Ok(())
    }
}
