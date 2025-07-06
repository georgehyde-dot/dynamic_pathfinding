use std::fmt;

#[derive(Debug, Clone)]
pub struct Statistics {
    pub total_moves: usize,
    pub num_obstacles: usize,
    pub num_walls: usize,
    pub route_efficiency: f64,
    pub optimal_path_length: usize,
}

#[derive(Debug, Clone)]
pub enum AlgorithmStats {
    AStar(usize),
    DStarLite(usize),
    Hybrid { a_star_calls: usize, d_star_calls: usize },
}

impl AlgorithmStats {
    pub fn total_calls(&self) -> usize {
        match self {
            AlgorithmStats::AStar(calls) => *calls,
            AlgorithmStats::DStarLite(calls) => *calls,
            AlgorithmStats::Hybrid { a_star_calls, d_star_calls } => a_star_calls + d_star_calls,
        }
    }
}

impl fmt::Display for AlgorithmStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AlgorithmStats::AStar(calls) => {
                writeln!(f, "A* Algorithm Statistics:")?;
                writeln!(f, "Total pathfinding calls: {}", calls)?;
            }
            AlgorithmStats::DStarLite(calls) => {
                writeln!(f, "D* Lite Algorithm Statistics:")?;
                writeln!(f, "Total pathfinding calls: {}", calls)?;
            }
            AlgorithmStats::Hybrid { a_star_calls, d_star_calls } => {
                let total = a_star_calls + d_star_calls;
                let a_star_percentage = if total > 0 {
                    (*a_star_calls as f64 / total as f64) * 100.0
                } else {
                    0.0
                };
                let d_star_percentage = if total > 0 {
                    (*d_star_calls as f64 / total as f64) * 100.0
                } else {
                    0.0
                };

                writeln!(f, "Hybrid A*/D* Algorithm Statistics:")?;
                writeln!(f, "Total pathfinding calls: {}", total)?;
                writeln!(f, "A* usage: {} calls ({:.1}%)", a_star_calls, a_star_percentage)?;
                writeln!(f, "D* Lite usage: {} calls ({:.1}%)", d_star_calls, d_star_percentage)?;
                
                if total > 0 {
                    if *a_star_calls == 1 && *d_star_calls > 0 {
                        writeln!(f, "✓ Optimal hybrid performance: A* used once for initial path, D* Lite handled all updates")?;
                    } else if *a_star_calls > 1 {
                        writeln!(f, "⚠ Multiple A* calls detected - may indicate significant environment changes")?;
                    } else if *d_star_calls == 0 {
                        writeln!(f, "⚠ Only A* was used - no incremental updates occurred")?;
                    }
                }
            }
        }
        Ok(())
    }
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
            self.route_efficiency = self.total_moves as f64 / self.optimal_path_length as f64;
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
