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
            field_of_view: 10,
            known_obstacles: HashSet::new(),
        }
    }

    pub fn observe(&mut self, grid: &Grid) {
        for x in (self.position.x.saturating_sub(self.field_of_view))..=(self.position.x + self.field_of_view) {
            for y in (self.position.y.saturating_sub(self.field_of_view))..=(self.position.y + self.field_of_view) {
                if x < grid.size && y < grid.size && grid.cells[x][y] == Cell::Obstacle {
                    self.known_obstacles.insert(Position { x, y });
                }
            }
        }
    }

    pub fn move_to(&mut self, new_pos: Position) {
        self.position = new_pos;
    }
}
