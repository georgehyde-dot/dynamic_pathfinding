use crate::algorithms::common::PathfindingAlgorithm;
use crate::grid::{Grid, Position};
use std::collections::{HashMap, HashSet, BinaryHeap};
use std::cmp::Ordering;

/// Represents the priority key for a node in the D* Lite priority queue.
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
        // Reversed comparison to make BinaryHeap a min-heap
        other.k1.cmp(&self.k1).then_with(|| other.k2.cmp(&self.k2))
    }
}

/// Implements the D* Lite pathfinding algorithm.
pub struct DStarLite {
    /// g(n): The actual cost incurred to reach node n
    g_scores: HashMap<Position, i32>,
    /// rhs(n): A one-step lookahead estimate of the cost to reach node n
    rhs_scores: HashMap<Position, i32>,
    /// Priority queue for processing nodes
    queue: BinaryHeap<(Key, Position)>,
    /// Key modifier for handling path cost changes
    k_m: i32,
    /// Current start position (changes as agent moves)
    start: Position,
    /// Goal position (fixed)
    goal: Position,
    /// Last known start position for k_m calculation
    last_start: Position,
    /// Track if initial computation is done
    initial_computation_done: bool,
}

impl DStarLite {
    /// Creates a new instance of the D* Lite algorithm.
    pub fn new(start: Position, goal: Position) -> Self {
        DStarLite {
            g_scores: HashMap::new(),
            rhs_scores: HashMap::new(),
            queue: BinaryHeap::new(),
            k_m: 0,
            start,
            goal,
            last_start: start,
            initial_computation_done: false,
        }
    }

    /// Initialize the algorithm
    fn initialize(&mut self) {
        if !self.initial_computation_done {
            println!("DEBUG: D* Lite initializing - Goal: {:?}, Start: {:?}", self.goal, self.start);
            // Initialize goal with rhs = 0 and add to queue
            self.rhs_scores.insert(self.goal, 0);
            self.queue.push((self.calculate_key(self.goal), self.goal));
            self.initial_computation_done = true;
            println!("DEBUG: D* Lite initialized - Queue size: {}, Goal rhs: 0", self.queue.len());
        } else {
            println!("DEBUG: D* Lite already initialized - Queue size: {}", self.queue.len());
        }
    }

    /// Calculates the heuristic value (Manhattan distance) from a position to the start.
    fn heuristic(&self, pos: Position) -> i32 {
        (pos.x as i32 - self.start.x as i32).abs() + (pos.y as i32 - self.start.y as i32).abs()
    }

    /// Calculates the key for a given position for the priority queue.
    fn calculate_key(&self, pos: Position) -> Key {
        let g = *self.g_scores.get(&pos).unwrap_or(&i32::MAX);
        let rhs = *self.rhs_scores.get(&pos).unwrap_or(&i32::MAX);
        let min_score = g.min(rhs);

        if min_score == i32::MAX {
            Key { k1: i32::MAX, k2: i32::MAX }
        } else {
            Key {
                k1: min_score.saturating_add(self.heuristic(pos)).saturating_add(self.k_m),
                k2: min_score,
            }
        }
    }

    /// Get the cost of moving from one position to another
    fn get_edge_cost(&self, _from: Position, to: Position, grid: &Grid, obstacles: &HashSet<Position>) -> i32 {
        // Check bounds
        if to.x >= grid.size || to.y >= grid.size {
            return i32::MAX;
        }
        
        // Check if the destination is blocked
        if obstacles.contains(&to) || grid.cells[to.x][to.y] == crate::grid::Cell::Wall {
            i32::MAX
        } else {
            1 // Standard movement cost
        }
    }

    /// Updates a vertex's rhs-value and its position in the priority queue.
    fn update_vertex(&mut self, pos: Position, grid: &Grid, obstacles: &HashSet<Position>) {
        // Update rhs value for non-goal nodes
        if pos != self.goal {
            let mut min_rhs = i32::MAX;
            
            // Check all predecessors (neighbors that can reach this position)
            for pred in grid.get_neighbors(&pos) {
                let g_pred = *self.g_scores.get(&pred).unwrap_or(&i32::MAX);
                let edge_cost = self.get_edge_cost(pred, pos, grid, obstacles);
                
                if g_pred != i32::MAX && edge_cost != i32::MAX {
                    let total_cost = g_pred.saturating_add(edge_cost);
                    min_rhs = min_rhs.min(total_cost);
                }
            }
            self.rhs_scores.insert(pos, min_rhs);
        }

        // Remove any existing entry for this position from the queue
        self.queue.retain(|(_, p)| *p != pos);

        // Add to the queue if the node is inconsistent (g != rhs)
        let g_val = *self.g_scores.get(&pos).unwrap_or(&i32::MAX);
        let rhs_val = *self.rhs_scores.get(&pos).unwrap_or(&i32::MAX);
        
        if g_val != rhs_val {
            self.queue.push((self.calculate_key(pos), pos));
        }
    }

    /// Check if the start node has been properly processed (has a finite g-value)
    fn start_is_processed(&self) -> bool {
        let start_g = *self.g_scores.get(&self.start).unwrap_or(&i32::MAX);
        start_g != i32::MAX
    }

    /// Check if start is truly consistent (g == rhs and both are finite)
    fn start_is_consistent(&self) -> bool {
        let start_g = *self.g_scores.get(&self.start).unwrap_or(&i32::MAX);
        let start_rhs = *self.rhs_scores.get(&self.start).unwrap_or(&i32::MAX);
        
        // Only consider it consistent if both values are finite and equal
        start_g != i32::MAX && start_rhs != i32::MAX && start_g == start_rhs
    }

    /// Main computation loop to process the priority queue
    fn compute_shortest_path(&mut self, grid: &Grid, obstacles: &HashSet<Position>) {
        println!("DEBUG: Starting compute_shortest_path - Queue size: {}", self.queue.len());
        
        let mut iterations = 0;
        let max_iterations = grid.size * grid.size * 4;

        while !self.queue.is_empty() && iterations < max_iterations {
            let start_key = self.calculate_key(self.start);
            let start_g = *self.g_scores.get(&self.start).unwrap_or(&i32::MAX);
            let start_rhs = *self.rhs_scores.get(&self.start).unwrap_or(&i32::MAX);
            
            if let Some((top_key, top_pos)) = self.queue.peek() {
                if iterations < 5 || iterations % 10 == 0 {  // Reduce debug spam
                    println!("DEBUG: Iteration {}: Top key: {:?} (pos: {:?}), Start key: {:?}, Start g: {}, Start rhs: {}", 
                             iterations, top_key, top_pos, start_key, start_g, start_rhs);
                }
            }
            
            // Modified stopping condition:
            // Continue while: 
            // 1. Top key < start key, OR
            // 2. Start is not consistent (using our improved check), OR  
            // 3. Start hasn't been processed yet (still has infinite g-value)
            let should_continue = if let Some((top_key, _)) = self.queue.peek() {
                *top_key < start_key || !self.start_is_consistent() || !self.start_is_processed()
            } else {
                false
            };
            
            if !should_continue {
                println!("DEBUG: Stopping computation - start is processed and consistent, and top key >= start key");
                break;
            }

            let (k_old, u) = self.queue.pop().unwrap();
            let k_new = self.calculate_key(u);

            if k_old < k_new {
                // Key has changed, reinsert with new key
                if iterations < 5 {
                    println!("DEBUG: Key changed for {:?}, reinserting", u);
                }
                self.queue.push((k_new, u));
            } else {
                let g_u = *self.g_scores.get(&u).unwrap_or(&i32::MAX);
                let rhs_u = *self.rhs_scores.get(&u).unwrap_or(&i32::MAX);
                
                if iterations < 5 {
                    println!("DEBUG: Processing {:?} - g: {}, rhs: {}", u, g_u, rhs_u);
                }

                if g_u > rhs_u {
                    // Locally underconsistent: set g = rhs and update successors
                    self.g_scores.insert(u, rhs_u);
                    
                    // Update all successors (neighbors this position can reach)
                    for successor in grid.get_neighbors(&u) {
                        self.update_vertex(successor, grid, obstacles);
                    }
                } else {
                    // Locally overconsistent: set g = infinity and update u and successors
                    self.g_scores.insert(u, i32::MAX);
                    self.update_vertex(u, grid, obstacles);
                    
                    for successor in grid.get_neighbors(&u) {
                        self.update_vertex(successor, grid, obstacles);
                    }
                }
            }
            
            iterations += 1;
        }
        
        let final_start_g = *self.g_scores.get(&self.start).unwrap_or(&i32::MAX);
        let final_start_rhs = *self.rhs_scores.get(&self.start).unwrap_or(&i32::MAX);
        println!("DEBUG: Finished compute_shortest_path - Iterations: {}, Start g: {}, Start rhs: {}, Queue size: {}", 
                 iterations, final_start_g, final_start_rhs, self.queue.len());
    }

    /// Extract the optimal path from current start to goal
    fn extract_path(&self, grid: &Grid, obstacles: &HashSet<Position>) -> Option<Vec<Position>> {
        println!("DEBUG: Starting extract_path from {:?} to {:?}", self.start, self.goal);
        
        // Check if start is reachable
        let start_g = *self.g_scores.get(&self.start).unwrap_or(&i32::MAX);
        if start_g == i32::MAX {
            println!("DEBUG: extract_path failed - start g-value is infinite");
            return None;
        }

        let mut path = vec![self.start];
        let mut current = self.start;
        let mut visited = HashSet::new();

        while current != self.goal {
            if visited.contains(&current) {
                println!("DEBUG: extract_path failed - cycle detected at {:?}", current);
                return None; // Cycle detected
            }
            visited.insert(current);
            
            let neighbors = grid.get_neighbors(&current);
            println!("DEBUG: At {:?}, neighbors: {:?}", current, neighbors);
            
            // Find the neighbor that leads to the goal most efficiently
            // Instead of just minimum g-value, consider g-value + edge cost + heuristic
            let next_step = neighbors
                .iter()
                .filter(|&pos| {
                    let edge_cost = self.get_edge_cost(current, *pos, grid, obstacles);
                    edge_cost != i32::MAX && !visited.contains(pos) // Don't revisit positions
                })
                .min_by_key(|&pos| {
                    let g_val = *self.g_scores.get(pos).unwrap_or(&i32::MAX);
                    let edge_cost = self.get_edge_cost(current, *pos, grid, obstacles);
                    let heuristic_to_goal = (pos.x as i32 - self.goal.x as i32).abs() + 
                                           (pos.y as i32 - self.goal.y as i32).abs();
                    
                    if g_val == i32::MAX || edge_cost == i32::MAX {
                        i32::MAX
                    } else {
                        // Use f-value (g + h) for better pathfinding
                        g_val.saturating_add(heuristic_to_goal)
                    }
                });

            if let Some(&next) = next_step {
                let next_g = *self.g_scores.get(&next).unwrap_or(&i32::MAX);
                println!("DEBUG: Next step: {:?} with g-value: {}", next, next_g);
                
                if next_g == i32::MAX {
                    println!("DEBUG: extract_path failed - next step has infinite g-value");
                    return None;
                }
                
                path.push(next);
                current = next;
                
                // Safety check to prevent infinite loops
                if path.len() > grid.size * grid.size {
                    println!("DEBUG: extract_path failed - path too long");
                    return None;
                }
            } else {
                println!("DEBUG: extract_path failed - no valid next step from {:?}", current);
                // Print g-values of all neighbors for debugging
                for neighbor in neighbors {
                    let g_val = *self.g_scores.get(&neighbor).unwrap_or(&i32::MAX);
                    let edge_cost = self.get_edge_cost(current, neighbor, grid, obstacles);
                    let is_visited = visited.contains(&neighbor);
                    println!("DEBUG: Neighbor {:?} - g: {}, edge_cost: {}, visited: {}", 
                             neighbor, g_val, edge_cost, is_visited);
                }
                
                // If we can't find a next step, try to backtrack or find alternative
                // Remove the cycle detection temporarily and try a different approach
                if let Some(alternative) = self.find_alternative_path(current, &visited, grid, obstacles) {
                    println!("DEBUG: Found alternative path from {:?} to {:?}", current, alternative);
                    path.push(alternative);
                    current = alternative;
                } else {
                    return None; // No valid next step and no alternative
                }
            }
        }

        println!("DEBUG: extract_path succeeded - path length: {}", path.len());
        Some(path)
    }

    /// Find an alternative path when stuck in a local minimum
    fn find_alternative_path(&self, current: Position, visited: &HashSet<Position>, 
                           grid: &Grid, obstacles: &HashSet<Position>) -> Option<Position> {
        let neighbors = grid.get_neighbors(&current);
        
        // Look for any unvisited neighbor that's not blocked, even if it has a higher g-value
        // This helps escape local minima
        let alternative = neighbors
            .iter()
            .filter(|&pos| {
                let edge_cost = self.get_edge_cost(current, *pos, grid, obstacles);
                edge_cost != i32::MAX && !visited.contains(pos)
            })
            .min_by_key(|&pos| {
                // Prefer positions closer to the goal (Manhattan distance)
                (pos.x as i32 - self.goal.x as i32).abs() + 
                (pos.y as i32 - self.goal.y as i32).abs()
            });

        alternative.copied()
    }

    /// Check if obstacles have changed since last computation
    fn check_obstacles_changed(&self, current_obstacles: &HashSet<Position>) -> bool {
        // For simplicity, assume obstacles might have changed if we have any
        // In a more sophisticated implementation, we'd track the previous obstacle set
        !current_obstacles.is_empty()
    }

    /// Update vertices affected by obstacle changes
    fn update_changed_obstacles(&mut self, grid: &Grid, obstacles: &HashSet<Position>) {
        // Update all positions that might be affected by obstacles
        let mut positions_to_update = HashSet::new();
        
        // Add obstacle positions and their neighbors
        for &obstacle_pos in obstacles {
            positions_to_update.insert(obstacle_pos);
            for neighbor in grid.get_neighbors(&obstacle_pos) {
                positions_to_update.insert(neighbor);
            }
        }
        
        // Also update positions around the current start
        positions_to_update.insert(self.start);
        for neighbor in grid.get_neighbors(&self.start) {
            positions_to_update.insert(neighbor);
        }
        
        // Update all affected vertices
        for pos in &positions_to_update {
            self.update_vertex(*pos, grid, obstacles);
        }
        
        println!("DEBUG: Updated {} vertices due to obstacle changes", positions_to_update.len());
    }
}

impl PathfindingAlgorithm for DStarLite {
    /// Finds a path from start to goal using the D* Lite algorithm.
    fn find_path(
        &mut self,
        grid: &Grid,
        start: Position,
        goal: Position,
        obstacles: &HashSet<Position>,
    ) -> Option<Vec<Position>> {
        println!("DEBUG: find_path called - Start: {:?}, Goal: {:?}, Obstacles: {:?}", start, goal, obstacles);
        
        // Handle goal changes by reinitializing
        if self.goal != goal {
            println!("DEBUG: Goal changed, reinitializing");
            *self = DStarLite::new(start, goal);
        }

        // Handle start changes
        if self.start != start {
            println!("DEBUG: Start changed from {:?} to {:?}", self.start, start);
            // Update k_m when start changes
            if self.initial_computation_done {
                self.k_m = self.k_m.saturating_add(self.heuristic(self.last_start));
                println!("DEBUG: Updated k_m to {}", self.k_m);
            }
            self.last_start = self.start;
            self.start = start;
            
            // When start changes, we need to update the vertex and recompute
            if self.initial_computation_done {
                self.update_vertex(self.start, grid, obstacles);
                println!("DEBUG: Updated start vertex after position change");
            }
        }

        // Initialize if not done yet
        self.initialize();

        // Always recompute when obstacles might have changed or start moved
        // Check if obstacles have changed by comparing with known obstacles
        let obstacles_changed = self.check_obstacles_changed(obstacles);
        if obstacles_changed {
            println!("DEBUG: Obstacles changed, updating affected vertices");
            self.update_changed_obstacles(grid, obstacles);
        }

        // Compute shortest path
        self.compute_shortest_path(grid, obstacles);
        
        // Extract and return the path
        let result = self.extract_path(grid, obstacles);
        println!("DEBUG: find_path returning: {:?}", result.as_ref().map(|p| format!("Some(path with {} steps)", p.len())));
        result
    }
}
