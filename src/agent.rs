use crate::grid::{Grid, Position, Cell};
use std::collections::HashSet;

pub struct Agent {
    pub position: Position,
    pub known_obstacles: HashSet<Position>,
    pub current_path: Option<Vec<Position>>,
    pub path_index: usize,
}

impl Agent {
    pub fn new(start: Position) -> Self {
        Agent {
            position: start,
            known_obstacles: HashSet::new(),
            current_path: None,
            path_index: 0,
        }
    }

    pub fn observe(&mut self, grid: &Grid) {
        // Observe obstacles around agent (within observation range)
        for neighbor in grid.get_neighbors(&self.position) {
            if grid.cells[neighbor.x][neighbor.y] == Cell::Obstacle {
                self.known_obstacles.insert(neighbor);
            }
        }
    }

    /// Move along the current path (more efficient than recalculating every step)
    pub fn move_along_path(&mut self) -> bool {
        if let Some(ref path) = self.current_path {
            if self.path_index + 1 < path.len() {
                self.path_index += 1;
                self.position = path[self.path_index];
                return true;
            }
        }
        false
    }

    /// Set a new path and reset the path index
    pub fn set_path(&mut self, path: Vec<Position>) {
        self.current_path = Some(path);
        self.path_index = 0;
        if let Some(ref path) = self.current_path {
            if !path.is_empty() {
                self.position = path[0];
            }
        }
    }

    pub fn get_next_step(&self) -> Option<Position> {
        if let Some(ref path) = self.current_path {
            if self.path_index + 1 < path.len() {
                Some(path[self.path_index + 1])
            } else {
                None // Reached end of path
            }
        } else {
            None // No path set
        }
    }

    pub fn move_to(&mut self, position: Position) {
        self.position = position;
        self.path_index += 1;
    }

    pub fn is_path_blocked(&self, grid: &Grid) -> bool {
        if let Some(next_pos) = self.get_next_step() {
            // Check if next step is blocked
            grid.cells[next_pos.x][next_pos.y] == Cell::Obstacle ||
            grid.cells[next_pos.x][next_pos.y] == Cell::Wall ||
            self.known_obstacles.contains(&next_pos)
        } else {
            false
        }
    }

    /// Check if current path needs recalculation (OPTIMIZED - only check next few steps)
    pub fn path_needs_recalculation(&self, grid: &Grid) -> bool {
        if let Some(ref path) = self.current_path {
            // Only check next 3-5 steps ahead, not entire path
            let check_ahead = 3.min(path.len().saturating_sub(self.path_index + 1));
            for i in 1..=check_ahead {
                if self.path_index + i < path.len() {
                    let pos = path[self.path_index + i];
                    if grid.cells[pos.x][pos.y] == Cell::Obstacle ||
                       grid.cells[pos.x][pos.y] == Cell::Wall ||
                       self.known_obstacles.contains(&pos) {
                        return true;
                    }
                }
            }
            false
        } else {
            true
        }
    }

    pub fn has_path(&self) -> bool {
        self.current_path.is_some()
    }

    pub fn get_current_path(&self) -> Option<&Vec<Position>> {
        self.current_path.as_ref()
    }

    pub fn get_path_progress(&self) -> (usize, usize) {
        if let Some(ref path) = self.current_path {
            (self.path_index, path.len())
        } else {
            (0, 0)
        }
    }

    /// Check if agent is at the goal
    pub fn is_at_goal(&self, goal: Position) -> bool {
        self.position == goal
    }

    pub fn clear_path(&mut self) {
        self.current_path = None;
        self.path_index = 0;
    }
}
