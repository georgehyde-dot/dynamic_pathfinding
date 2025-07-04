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
        // Get the current field of view bounds
        let min_x = self.position.x.saturating_sub(self.field_of_view);
        let max_x = (self.position.x + self.field_of_view).min(grid.size - 1);
        let min_y = self.position.y.saturating_sub(self.field_of_view);
        let max_y = (self.position.y + self.field_of_view).min(grid.size - 1);

        // Remove all obstacles that are within the current field of view
        // (we'll re-add the ones that are still there)
        let old_count = self.known_obstacles.len();
        self.known_obstacles.retain(|pos| {
            // Keep obstacles that are outside the current field of view
            !(pos.x >= min_x && pos.x <= max_x && pos.y >= min_y && pos.y <= max_y)
        });

        // Add all obstacles that are currently visible
        for x in min_x..=max_x {
            for y in min_y..=max_y {
                if grid.cells[x][y] == Cell::Obstacle {
                    self.known_obstacles.insert(Position { x, y });
                }
            }
        }

        let new_count = self.known_obstacles.len();
        println!("DEBUG: Agent at {:?} observing area ({},{}) to ({},{}). Known obstacles: {} -> {}", 
                 self.position, min_x, min_y, max_x, max_y, old_count, new_count);
    }

    pub fn move_to(&mut self, new_pos: Position) {
        println!("DEBUG: Agent moving from {:?} to {:?}", self.position, new_pos);
        self.position = new_pos;
    }

    /// Clear knowledge of obstacles that are no longer present (called from simulation)
    /// This is a backup method, but the main logic should be in observe()
    pub fn update_known_obstacles(&mut self, grid: &Grid) {
        let initial_count = self.known_obstacles.len();
        self.known_obstacles.retain(|pos| {
            grid.cells[pos.x][pos.y] == Cell::Obstacle
        });
        let final_count = self.known_obstacles.len();
        
        if initial_count != final_count {
            println!("DEBUG: Agent updated known obstacles: {} -> {} (removed {} obstacles)", 
                     initial_count, final_count, initial_count - final_count);
        }
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
