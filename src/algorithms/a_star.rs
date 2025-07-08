use crate::algorithms::common::PathfindingAlgorithm;
use crate::grid::{Grid, Position, Cell};
use pathfinding::prelude::astar;
use std::collections::HashSet;

/// Implements the A* pathfinding algorithm using the `pathfinding` crate.
#[derive(Default)]
pub struct AStar;

impl AStar {
    /// Creates a new instance of the A* algorithm provider.
    pub fn new() -> Self {
        AStar
    }
}

impl PathfindingAlgorithm for AStar {
    /// Finds a path from start to goal using the A* algorithm.
    ///
    /// # Arguments
    ///
    /// * `grid` - The simulation grid.
    /// * `start` - The starting position.
    /// * `goal` - The goal position.
    /// * `obstacles` - A set of known obstacle positions to avoid.
    ///
    /// # Returns
    ///
    /// An `Option` containing a `Vec<Position>` representing the path, or `None` if no path is found.
    fn find_path(
        &mut self,
        grid: &Grid,
        start: Position,
        goal: Position,
        obstacles: &HashSet<Position>,
    ) -> Option<Vec<Position>> {
        let result = astar(
            &start,
            |p| {
                // Successors are valid neighbors that are not known obstacles.
                grid.get_neighbors(p)
                    .into_iter()
                    .filter(|neighbor| {
                        // The agent can't move through walls or known dynamic obstacles.
                        grid.cells[neighbor.x][neighbor.y] != Cell::Wall && !obstacles.contains(neighbor)
                    })
                    .map(|successor| (successor, 1)) // Cost of moving to a neighbor is 1.
                    .collect::<Vec<_>>()
            },
            |p| {
                // Heuristic: Manhattan distance to the goal.
                ((p.x as i32 - goal.x as i32).abs() + (p.y as i32 - goal.y as i32).abs()) as u32
            },
            |p| *p == goal, // Success condition: we've reached the goal.
        );

        // The result from `astar` is a tuple `(path, cost)`. We only need the path.
        result.map(|(path, _)| path)
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}
