use rand::Rng;
use std::collections::HashSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Position {
    pub x: usize,
    pub y: usize,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Cell {
    Empty,
    Wall,
    Obstacle,
}

pub struct Grid {
    pub size: usize,
    pub cells: Vec<Vec<Cell>>,
    pub start: Position,
    pub goal: Position,
}

impl Grid {
    pub fn new(size: usize, num_walls: usize) -> Self {
        let mut cells = vec![vec![Cell::Empty; size]; size];
        let mut rng = rand::thread_rng();
        
        // Generate random start and goal positions
        let start = Position { 
            x: rng.gen_range(0..size/2), 
            y: rng.gen_range(0..size/2) 
        };
        let goal = Position { 
            x: rng.gen_range(size/2..size), 
            y: rng.gen_range(size/2..size) 
        };

        // Place walls randomly, ensuring we don't block start/goal
        let mut walls_placed = 0;
        let mut attempts = 0;
        while walls_placed < num_walls && attempts < num_walls * 3 {
            let x = rng.gen_range(0..size);
            let y = rng.gen_range(0..size);
            let pos = Position { x, y };
            
            if pos != start && pos != goal && cells[x][y] == Cell::Empty {
                cells[x][y] = Cell::Wall;
                walls_placed += 1;
            }
            attempts += 1;
        }

        Grid {
            size,
            cells,
            start,
            goal,
        }
    }

    pub fn get_neighbors(&self, pos: &Position) -> Vec<Position> {
        let mut neighbors = Vec::new();
        let (x, y) = (pos.x as i32, pos.y as i32);

        for (dx, dy) in &[(0, 1), (0, -1), (1, 0), (-1, 0)] {
            let nx = x + dx;
            let ny = y + dy;

            if nx >= 0 && nx < self.size as i32 && ny >= 0 && ny < self.size as i32 {
                let next_pos = Position { x: nx as usize, y: ny as usize };
                if self.cells[next_pos.x][next_pos.y] != Cell::Wall {
                    neighbors.push(next_pos);
                }
            }
        }
        neighbors
    }

    /// Print a visual representation of the grid with enhanced formatting
    pub fn print_grid(&self, agent_pos: Option<Position>) {
        println!("Legend: S=Start, G=Goal, A=Agent, #=Wall, O=Obstacle, .=Empty");
        
        // Print column numbers header
        print!("   ");
        for x in 0..self.size {
            print!("{:2}", x % 10);
        }
        println!();
        
        for y in 0..self.size {
            // Print row number
            print!("{:2} ", y);
            
            for x in 0..self.size {
                let pos = Position { x, y };
                let char = if Some(pos) == agent_pos {
                    'A'
                } else if pos == self.start {
                    'S'
                } else if pos == self.goal {
                    'G'
                } else {
                    match self.cells[x][y] {
                        Cell::Wall => '#',
                        Cell::Obstacle => 'O',
                        Cell::Empty => '.',
                    }
                };
                print!("{} ", char);
            }
            println!();
        }
        println!();
    }
}
