use crate::algorithms::common::PathfindingAlgorithm;
use crate::algorithms::a_star::AStar;
use crate::algorithms::d_star_lite_simple::DStarLiteSimple;
use crate::grid::{Grid, Position};
use std::collections::HashSet;

/// Hybrid algorithm that uses A* for initial path finding and D* Lite Simple for updates
pub struct HybridAStarDStar {
    a_star: AStar,
    d_star_lite_simple: DStarLiteSimple,
    initial_path_found: bool,
    last_start: Position,
    last_goal: Position,
    last_obstacles: HashSet<Position>,
    // Add usage tracking
    a_star_usage_count: usize,
    d_star_usage_count: usize,
}

impl HybridAStarDStar {
    pub fn new(start: Position, goal: Position) -> Self {
        HybridAStarDStar {
            a_star: AStar::new(),
            d_star_lite_simple: DStarLiteSimple::new(),
            initial_path_found: false,
            last_start: start,
            last_goal: goal,
            last_obstacles: HashSet::new(),
            a_star_usage_count: 0,
            d_star_usage_count: 0,
        }
    }

    /// Get usage statistics
    pub fn get_usage_stats(&self) -> (usize, usize) {
        (self.a_star_usage_count, self.d_star_usage_count)
    }

    /// Print detailed usage statistics
    pub fn print_usage_stats(&self) {
        let total_calls = self.a_star_usage_count + self.d_star_usage_count;
        let a_star_percentage = if total_calls > 0 {
            (self.a_star_usage_count as f64 / total_calls as f64) * 100.0
        } else {
            0.0
        };
        let d_star_percentage = if total_calls > 0 {
            (self.d_star_usage_count as f64 / total_calls as f64) * 100.0
        } else {
            0.0
        };

        println!("\n=== HYBRID ALGORITHM USAGE STATISTICS ===");
        println!("Total pathfinding calls: {}", total_calls);
        println!("A* usage: {} calls ({:.1}%)", self.a_star_usage_count, a_star_percentage);
        println!("D* Lite Simple usage: {} calls ({:.1}%)", self.d_star_usage_count, d_star_percentage);
        println!();
        
        if total_calls > 0 {
            if self.a_star_usage_count == 1 && self.d_star_usage_count > 0 {
                println!("✓ Optimal hybrid performance: A* used once for initial path, D* Lite Simple handled all updates");
            } else if self.a_star_usage_count > 1 {
                println!("⚠ Multiple A* calls detected - may indicate significant environment changes");
                println!("  This could be due to goal changes or major start position jumps");
            } else if self.d_star_usage_count == 0 {
                println!("⚠ Only A* was used - no incremental updates occurred");
            }
        }
    }

    /// Check if we need to use A* (first run or major changes)
    fn should_use_astar(&self, start: Position, goal: Position, obstacles: &HashSet<Position>) -> bool {
        // Use A* if:
        // 1. No initial path found yet
        // 2. Goal changed
        // 3. Start changed significantly (more than a few steps)
        if !self.initial_path_found || self.last_goal != goal {
            return true;
        }
        
        // Check if start moved significantly
        let start_distance = (start.x as i32 - self.last_start.x as i32).abs() + 
                           (start.y as i32 - self.last_start.y as i32).abs();
        if start_distance > 3 {
            return true;
        }
        
        // Check if obstacles changed significantly
        let obstacles_changed = obstacles != &self.last_obstacles;
        let major_obstacle_change = obstacles_changed && 
            (obstacles.len() as i32 - self.last_obstacles.len() as i32).abs() > 5;
        
        major_obstacle_change
    }
}

impl PathfindingAlgorithm for HybridAStarDStar {
    fn find_path(
        &mut self,
        grid: &Grid,
        start: Position,
        goal: Position,
        obstacles: &HashSet<Position>,
    ) -> Option<Vec<Position>> {
        // Check if we should use A* for this computation
        if self.should_use_astar(start, goal, obstacles) {
            // Increment A* usage counter
            self.a_star_usage_count += 1;
            
            // Use A* to find initial path
            if let Some(path) = self.a_star.find_path(grid, start, goal, obstacles) {
                // Update tracking variables
                self.last_start = start;
                self.last_goal = goal;
                self.last_obstacles = obstacles.clone();
                self.initial_path_found = true;
                
                return Some(path);
            } else {
                return None;
            }
        } else {
            // Increment D* Lite Simple usage counter
            self.d_star_usage_count += 1;
            
            // Use D* Lite Simple for incremental updates
            let result = self.d_star_lite_simple.find_path(grid, start, goal, obstacles);
            
            // Update tracking variables
            self.last_start = start;
            self.last_obstacles = obstacles.clone();
            
            if let Some(ref _path) = result {
                return result;
            } else {
                // Fallback to A* if D* Lite Simple fails
                self.a_star_usage_count += 1;
                let fallback_result = self.a_star.find_path(grid, start, goal, obstacles);
                return fallback_result;
            }
        }
    }
    
    fn get_usage_stats(&self) -> (usize, usize) {
        (self.a_star_usage_count, self.d_star_usage_count)
    }
}