use crate::grid::{Grid, Position};
use std::collections::HashSet;
use std::any::Any;

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
    
    /// Update environment (for incremental algorithms like D* Lite)
    fn update_environment(&mut self, _grid: &Grid, _obstacles: &HashSet<Position>) {
        // Default: do nothing (most algorithms don't need this)
    }

    fn as_any_mut(&mut self) -> &mut dyn Any;
}
