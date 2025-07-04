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
}
