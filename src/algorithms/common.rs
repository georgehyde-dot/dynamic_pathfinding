use crate::grid::{Grid, Position};
use std::collections::HashSet;

pub trait PathfindingAlgorithm {
    fn find_path(
        &mut self,
        grid: &Grid,
        start: Position,
        goal: Position,
        obstacles: &HashSet<Position>,
    ) -> Option<Vec<Position>>;
    
    /// Get algorithm usage statistics (for hybrid algorithms)
    fn get_usage_stats(&self) -> (usize, usize) {
        (0, 0)  // Default: no breakdown available
    }
}
