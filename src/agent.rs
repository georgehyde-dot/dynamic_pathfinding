use crate::grid::{Grid, Position, Cell};
use std::collections::HashSet;

pub struct Agent {
    pub position: Position,
    pub field_of_view: usize,
    pub known_obstacles: HashSet<Position>,
}

impl Agent {
    pub fn new(start_pos: Position) -> Self {
        Agent {
            position: start_pos,
            field_of_view: 3, // Reasonable field of view
            known_obstacles: HashSet::new(),
        }
    }

    pub fn observe(&mut self, grid: &Grid) {
        // Clear all known obstacles - we only keep what we can currently see
        self.known_obstacles.clear();

        // Get the current field of view bounds
        let min_x = self.position.x.saturating_sub(self.field_of_view);
        let max_x = (self.position.x + self.field_of_view).min(grid.size - 1);
        let min_y = self.position.y.saturating_sub(self.field_of_view);
        let max_y = (self.position.y + self.field_of_view).min(grid.size - 1);

        // Add all obstacles and walls that are currently visible
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                if grid.cells[x][y] == Cell::Obstacle || grid.cells[x][y] == Cell::Wall {
                    self.known_obstacles.insert(Position { x, y });
                }
            }
        }
    }

    pub fn move_to(&mut self, new_pos: Position) {
        self.position = new_pos;
    }

    /// Clear knowledge of obstacles that are no longer present (called from simulation)
    /// This is a backup method, but the main logic should be in observe()
    pub fn update_known_obstacles(&mut self, grid: &Grid) {
        self.known_obstacles.retain(|pos| {
            grid.cells[pos.x][pos.y] == Cell::Obstacle || grid.cells[pos.x][pos.y] == Cell::Wall
        });
    }

    /// Get current field of view bounds for debugging
    pub fn get_field_of_view_bounds(&self, grid: &Grid) -> (usize, usize, usize, usize) {
        let min_x = self.position.x.saturating_sub(self.field_of_view);
        let max_x = (self.position.x + self.field_of_view).min(grid.size - 1);
        let min_y = self.position.y.saturating_sub(self.field_of_view);
        let max_y = (self.position.y + self.field_of_view).min(grid.size - 1);
        (min_x, max_x, min_y, max_y)
    }
}
