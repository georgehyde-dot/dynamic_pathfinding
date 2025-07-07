use crate::algorithms::common::PathfindingAlgorithm;
use crate::grid::{Grid, Position, Cell};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Ordering;
use rustc_hash::FxHashMap;

#[derive(Clone, Copy, PartialEq, Debug)]
struct Key {
    k1: i32,
    k2: i32,
}

impl Eq for Key {}

impl PartialOrd for Key {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Key {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for min-heap behavior
        match other.k1.cmp(&self.k1) {
            Ordering::Equal => other.k2.cmp(&self.k2),
            other => other,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
struct QueueEntry {
    key: Key,
    pos: Position,
    hash_code: u64,
}

impl Eq for QueueEntry {}

impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for QueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.key.cmp(&other.key)
    }
}

#[derive(Clone, Copy, Debug)]
struct CellData {
    g: i32,
    rhs: i32,
}

impl Default for CellData {
    fn default() -> Self {
        CellData { g: i32::MAX, rhs: i32::MAX }
    }
}

pub struct DStarLiteSimple {
    initialized: bool,
    s_start: Position,
    s_goal: Position,
    s_last: Position,
    k_m: i32,
}

impl DStarLiteSimple {
    pub fn new() -> Self {
        DStarLiteSimple {
            initialized: false,
            s_start: Position { x: 0, y: 0 },
            s_goal: Position { x: 0, y: 0 },
            s_last: Position { x: 0, y: 0 },
            k_m: 0,
        }
    }
}

impl PathfindingAlgorithm for DStarLiteSimple {
    fn find_path(
        &mut self,
        grid: &Grid,
        start: Position,
        goal: Position,
        obstacles: &HashSet<Position>,
    ) -> Option<Vec<Position>> {
        // Handle initialization and goal changes
        if !self.initialized || self.s_goal != goal {
            self.s_start = start;
            self.s_goal = goal;
            self.s_last = start;
            self.k_m = 0;
            self.initialized = true;
        } else if self.s_start != start {
            // Update k_m for start position changes
            self.k_m = self.k_m.saturating_add(
                (self.s_last.x as i32 - start.x as i32).abs() + 
                (self.s_last.y as i32 - start.y as i32).abs()
            );
            self.s_start = start;
            self.s_last = start;
        }

        // Main D* Lite algorithm - simplified and inlined
        let mut open_queue = BinaryHeap::new();
        let mut cell_data: FxHashMap<Position, CellData> = FxHashMap::default();
        let mut queue_hash: FxHashMap<Position, u64> = FxHashMap::default();
        let mut hash_counter = 0u64;

        // Helper functions
        let heuristic = |a: Position, b: Position| -> i32 {
            (a.x as i32 - b.x as i32).abs() + (a.y as i32 - b.y as i32).abs()
        };

        let get_g = |cell_data: &FxHashMap<Position, CellData>, pos: Position| -> i32 {
            cell_data.get(&pos).map_or(i32::MAX, |data| data.g)
        };

        let get_rhs = |cell_data: &FxHashMap<Position, CellData>, pos: Position| -> i32 {
            if pos == goal {
                0
            } else {
                cell_data.get(&pos).map_or(i32::MAX, |data| data.rhs)
            }
        };

        let cost = |a: Position, b: Position| -> i32 {
            if b.x >= grid.size || b.y >= grid.size {
                return i32::MAX;
            }
            if obstacles.contains(&b) || grid.cells[b.x][b.y] == Cell::Wall {
                return i32::MAX;
            }
            1
        };

        let calculate_key = |pos: Position, g: i32, rhs: i32, k_m: i32| -> Key {
            let min_val = g.min(rhs);
            if min_val == i32::MAX {
                Key { k1: i32::MAX, k2: i32::MAX }
            } else {
                Key {
                    k1: min_val.saturating_add(heuristic(pos, start)).saturating_add(k_m),
                    k2: min_val,
                }
            }
        };

        let mut insert = |pos: Position, open_queue: &mut BinaryHeap<QueueEntry>, 
                         queue_hash: &mut FxHashMap<Position, u64>, hash_counter: &mut u64,
                         cell_data: &FxHashMap<Position, CellData>, k_m: i32| {
            let g = get_g(cell_data, pos);
            let rhs = get_rhs(cell_data, pos);
            let key = calculate_key(pos, g, rhs, k_m);
            *hash_counter += 1;
            let hash_code = *hash_counter;
            
            queue_hash.insert(pos, hash_code);
            open_queue.push(QueueEntry { key, pos, hash_code });
        };

        // Initialize goal
        cell_data.insert(goal, CellData { g: i32::MAX, rhs: 0 });
        insert(goal, &mut open_queue, &mut queue_hash, &mut hash_counter, &cell_data, self.k_m);

        // Main computation loop
        let mut iterations = 0;
        'outer: while !open_queue.is_empty() && iterations < 80000 {
            // Check termination condition
            let start_g = get_g(&cell_data, start);
            let start_rhs = get_rhs(&cell_data, start);
            let start_key = calculate_key(start, start_g, start_rhs, self.k_m);
            
            if let Some(top_entry) = open_queue.peek() {
                // Fixed termination condition: only terminate if start has valid g value
                if top_entry.key >= start_key && start_rhs == start_g && start_g != i32::MAX {
                    break;
                }
            }

            // Lazy deletion - find valid entry
            let u = loop {
                if let Some(entry) = open_queue.pop() {
                    if queue_hash.get(&entry.pos).map_or(false, |&h| h == entry.hash_code) {
                        queue_hash.remove(&entry.pos);
                        break entry;
                    }
                } else {
                    break 'outer; // No more entries, exit main loop
                }
            };

            let u_g = get_g(&cell_data, u.pos);
            let u_rhs = get_rhs(&cell_data, u.pos);
            let u_key_new = calculate_key(u.pos, u_g, u_rhs, self.k_m);

            if u.key < u_key_new {
                insert(u.pos, &mut open_queue, &mut queue_hash, &mut hash_counter, &cell_data, self.k_m);
            } else if u_g > u_rhs {
                // Update g value
                cell_data.entry(u.pos).or_default().g = u_rhs;
                
                // Update predecessors
                let neighbors = grid.get_neighbors(&u.pos);
                for neighbor in neighbors {
                    if neighbor != goal {
                        let old_rhs = get_rhs(&cell_data, neighbor);
                        let mut min_rhs = i32::MAX;
                        for succ in grid.get_neighbors(&neighbor) {
                            let edge_cost = cost(neighbor, succ);
                            let g_succ = get_g(&cell_data, succ);
                            if edge_cost != i32::MAX && g_succ != i32::MAX {
                                min_rhs = min_rhs.min(edge_cost.saturating_add(g_succ));
                            }
                        }
                        cell_data.entry(neighbor).or_default().rhs = min_rhs;
                        
                        let neighbor_g = get_g(&cell_data, neighbor);
                        let neighbor_rhs = get_rhs(&cell_data, neighbor);
                        
                        queue_hash.remove(&neighbor);
                        if neighbor_g != neighbor_rhs {
                            insert(neighbor, &mut open_queue, &mut queue_hash, &mut hash_counter, &cell_data, self.k_m);
                        }
                    }
                }
            } else {
                // Set g to infinity
                cell_data.entry(u.pos).or_default().g = i32::MAX;
                
                // Update u and its predecessors
                let mut to_update = grid.get_neighbors(&u.pos);
                to_update.push(u.pos);
                
                for vertex in to_update {
                    if vertex != goal {
                        let old_rhs = get_rhs(&cell_data, vertex);
                        let mut min_rhs = i32::MAX;
                        for succ in grid.get_neighbors(&vertex) {
                            let edge_cost = cost(vertex, succ);
                            let g_succ = get_g(&cell_data, succ);
                            if edge_cost != i32::MAX && g_succ != i32::MAX {
                                min_rhs = min_rhs.min(edge_cost.saturating_add(g_succ));
                            }
                        }
                        cell_data.entry(vertex).or_default().rhs = min_rhs;
                        
                        let vertex_g = get_g(&cell_data, vertex);
                        let vertex_rhs = get_rhs(&cell_data, vertex);
                        
                        queue_hash.remove(&vertex);
                        if vertex_g != vertex_rhs {
                            insert(vertex, &mut open_queue, &mut queue_hash, &mut hash_counter, &cell_data, self.k_m);
                        }
                    }
                }
            }
            
            iterations += 1;
        }

        // Extract path - check if we have a valid path first
        if get_g(&cell_data, start) == i32::MAX {
            return None;
        }

        let mut path = vec![start];
        let mut current = start;
        
        while current != goal {
            let neighbors = grid.get_neighbors(&current);
            
            let next = neighbors
                .iter()
                .filter(|&neighbor| cost(current, *neighbor) != i32::MAX)
                .min_by_key(|&neighbor| {
                    let edge_cost = cost(current, *neighbor);
                    let g_neighbor = get_g(&cell_data, *neighbor);
                    if edge_cost == i32::MAX || g_neighbor == i32::MAX {
                        i32::MAX
                    } else {
                        edge_cost.saturating_add(g_neighbor)
                    }
                });
            
            if let Some(&next_pos) = next {
                path.push(next_pos);
                current = next_pos;
                
                if path.len() > grid.size * grid.size {
                    return None;
                }
            } else {
                return None;
            }
        }
        
        Some(path)
    }
}