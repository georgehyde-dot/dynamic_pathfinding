use crate::algorithms::common::PathfindingAlgorithm;
use crate::grid::{Grid, Position, Cell};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

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
        // For min-heap behavior with BinaryHeap (which is max-heap)
        // We reverse the comparison
        match other.k1.cmp(&self.k1) {
            Ordering::Equal => other.k2.cmp(&self.k2),
            other => other,
        }
    }
}

/// Implements the D* Lite pathfinding algorithm based on the 2002 paper by S. Koenig and M. Likhachev.
pub struct DStarLite {
    pub g_scores: HashMap<Position, i32>,      // Make public for hybrid access
    pub rhs_scores: HashMap<Position, i32>,    // Make public for hybrid access
    queue: BinaryHeap<(Key, Position, u64)>, // Priority queue U with generation counter
    vertex_generations: HashMap<Position, u64>, // Track current generation for each vertex
    current_generation: u64,               // Current generation counter
    k_m: i32,                              // Key modifier
    pub s_start: Position,                     // Make public for hybrid access
    pub s_goal: Position,                      // Goal position
    s_last: Position,                      // Last start position
    edge_costs: HashMap<(Position, Position), i32>, // c(u,v) edge costs
    pub initialized: bool,                     // Track if algorithm has been initialized
    pub last_known_obstacles: HashSet<Position>,  // Track what obstacles we've seen
    pub last_start: Position,                     // Track last start position
}

impl DStarLite {
    /// Creates a new instance of the D* Lite algorithm.
    pub fn new(start: Position, goal: Position) -> Self {
        DStarLite {
            g_scores: HashMap::new(),
            rhs_scores: HashMap::new(),
            queue: BinaryHeap::new(),
            vertex_generations: HashMap::new(),
            current_generation: 0,
            k_m: 0,
            s_start: start,
            s_goal: goal,
            s_last: start,
            edge_costs: HashMap::new(),
            initialized: false,
            last_known_obstacles: HashSet::new(),  // Initialize properly
            last_start: start,                     // Initialize properly
        }
    }
    

    /// procedure CalculateKey(s) - line 01'
    fn calculate_key(&self, s: Position) -> Key {
        let g_s = *self.g_scores.get(&s).unwrap_or(&i32::MAX);
        let rhs_s = *self.rhs_scores.get(&s).unwrap_or(&i32::MAX);
        let min_val = g_s.min(rhs_s);
        
        if min_val == i32::MAX {
            Key { k1: i32::MAX, k2: i32::MAX }
        } else {
            Key {
                k1: min_val.saturating_add(self.h(s, self.s_start)).saturating_add(self.k_m),
                k2: min_val,
            }
        }
    }

    /// Heuristic function h(s1, s2) - Manhattan distance
    fn h(&self, s1: Position, s2: Position) -> i32 {
        (s1.x as i32 - s2.x as i32).abs() + (s1.y as i32 - s2.y as i32).abs()
    }

    /// Get edge cost c(u, v)
    fn c(&self, u: Position, v: Position, grid: &Grid, obstacles: &HashSet<Position>) -> i32 {
        // Check if edge exists in our stored costs first
        if let Some(&cost) = self.edge_costs.get(&(u, v)) {
            return cost;
        }

        // Check bounds
        if v.x >= grid.size || v.y >= grid.size {
            return i32::MAX;
        }

        // Check if destination is blocked
        if obstacles.contains(&v) || grid.cells[v.x][v.y] == Cell::Wall {
            i32::MAX
        } else {
            1 // Standard movement cost
        }
    }

    /// Get successors of position s
    fn succ(&self, s: Position, grid: &Grid) -> Vec<Position> {
        grid.get_neighbors(&s)
    }

    /// Get predecessors of position s  
    fn pred(&self, s: Position, grid: &Grid) -> Vec<Position> {
        grid.get_neighbors(&s) // In grid world, predecessors = successors
    }

    /// procedure Initialize() - lines 02'-06'
    fn initialize(&mut self) {
        // Clear all data structures
        self.queue.clear();
        self.vertex_generations.clear();
        self.current_generation = 0;
        self.k_m = 0;
        self.g_scores.clear();
        self.rhs_scores.clear();
        
        // line 05': rhs(s_goal) = 0
        self.rhs_scores.insert(self.s_goal, 0);
        
        // line 06': U.Insert(s_goal, CalculateKey(s_goal))
        let key = self.calculate_key(self.s_goal);
        self.current_generation += 1;
        self.vertex_generations.insert(self.s_goal, self.current_generation);
        self.queue.push((key, self.s_goal, self.current_generation));
        
        self.initialized = true;
    }

    /// procedure UpdateVertex(u) - lines 07'-09' with lazy deletion
    fn update_vertex(&mut self, u: Position, grid: &Grid, obstacles: &HashSet<Position>) {
        let g_u = *self.g_scores.get(&u).unwrap_or(&i32::MAX);
        
        // Calculate new rhs(u) if u != s_goal
        if u != self.s_goal {
            let mut min_rhs = i32::MAX;
            let successors = self.succ(u, grid);
            
            for s_prime in successors {
                let cost = self.c(u, s_prime, grid, obstacles);
                let g_s_prime = *self.g_scores.get(&s_prime).unwrap_or(&i32::MAX);
                
                if cost != i32::MAX && g_s_prime != i32::MAX {
                    let total_cost = cost.saturating_add(g_s_prime);
                    min_rhs = min_rhs.min(total_cost);
                }
            }
            
            self.rhs_scores.insert(u, min_rhs);
        }
        
        let rhs_u = *self.rhs_scores.get(&u).unwrap_or(&i32::MAX);
        
        // Invalidate old entries by incrementing generation
        self.current_generation += 1;
        self.vertex_generations.insert(u, self.current_generation);
        
        // Insert u if it's inconsistent
        if g_u != rhs_u {
            let key = self.calculate_key(u);
            self.queue.push((key, u, self.current_generation));
        }
    }

    /// procedure ComputeShortestPath() - lines 10'-20' with lazy deletion
    fn compute_shortest_path(&mut self, grid: &Grid, obstacles: &HashSet<Position>) {
        while !self.queue.is_empty() {
            // Skip invalid entries using lazy deletion
            let (k_old, u) = loop {
                if let Some((k, pos, gen)) = self.queue.pop() {
                    // Check if this entry is still valid
                    if *self.vertex_generations.get(&pos).unwrap_or(&0) == gen {
                        break (k, pos);
                    }
                    // Skip this entry - it's been invalidated
                } else {
                    return; // Queue is empty
                }
            };
            
            // Check termination condition
            let start_key = self.calculate_key(self.s_start);
            let rhs_start = *self.rhs_scores.get(&self.s_start).unwrap_or(&i32::MAX);
            let g_start = *self.g_scores.get(&self.s_start).unwrap_or(&i32::MAX);

            let top_less_than_start = self.key_less_than(k_old, start_key);
            let start_inconsistent = rhs_start != g_start;
            
            if !top_less_than_start && !start_inconsistent {
                // Put the item back and break
                self.current_generation += 1;
                self.vertex_generations.insert(u, self.current_generation);
                self.queue.push((k_old, u, self.current_generation));
                break;
            }

            // Check if key has changed
            let k_new = self.calculate_key(u);
            if self.key_less_than(k_old, k_new) {
                self.current_generation += 1;
                self.vertex_generations.insert(u, self.current_generation);
                self.queue.push((k_new, u, self.current_generation));
                continue;
            }

            let g_u = *self.g_scores.get(&u).unwrap_or(&i32::MAX);
            let rhs_u = *self.rhs_scores.get(&u).unwrap_or(&i32::MAX);

            if g_u > rhs_u {
                // Make vertex consistent
                self.g_scores.insert(u, rhs_u);
                
                // Update all predecessors
                let predecessors = self.pred(u, grid);
                for s in predecessors {
                    self.update_vertex(s, grid, obstacles);
                }
            } else {
                // Set g(u) to infinity
                self.g_scores.insert(u, i32::MAX);
                
                // Update all predecessors and u itself
                let mut vertices_to_update = self.pred(u, grid);
                vertices_to_update.push(u);
                
                for s in vertices_to_update {
                    self.update_vertex(s, grid, obstacles);
                }
            }
        }
    }

    /// Update edge costs when obstacles change
    pub fn update_edge_costs(&mut self, grid: &Grid, obstacles: &HashSet<Position>) {
        self.edge_costs.clear();
        
        for x in 0..grid.size {
            for y in 0..grid.size {
                let pos = Position { x, y };
                for neighbor in grid.get_neighbors(&pos) {
                    let cost = if obstacles.contains(&neighbor) || 
                                  grid.cells[neighbor.x][neighbor.y] == Cell::Wall {
                        i32::MAX
                    } else {
                        1
                    };
                    self.edge_costs.insert((pos, neighbor), cost);
                }
            }
        }
    }
}

impl PathfindingAlgorithm for DStarLite {
    fn find_path(
        &mut self,
        grid: &Grid,
        start: Position,
        goal: Position,
        obstacles: &HashSet<Position>,
    ) -> Option<Vec<Position>> {
        // Only reinitialize if goal changed
        if !self.initialized || self.s_goal != goal {
            self.s_goal = goal;
            self.s_start = start;
            self.s_last = start;
            self.initialize();
            self.update_edge_costs(grid, obstacles);
            self.compute_shortest_path(grid, obstacles);
            self.last_known_obstacles = obstacles.clone();
        } else {
            // For incremental updates, only update what changed
            let obstacles_changed = obstacles != &self.last_known_obstacles;
            let start_changed = self.s_start != start;
            
            // Only update if something actually changed
            if start_changed || obstacles_changed {
                if start_changed {
                    self.s_last = self.s_start;
                    self.s_start = start;
                    self.k_m = self.k_m.saturating_add(self.h(self.s_last, self.s_start));
                }
                
                if obstacles_changed {
                    // Use incremental update instead of full rebuild
                    self.update_edge_costs_incremental(grid, obstacles);
                }
                
                self.compute_shortest_path(grid, obstacles);
                self.last_known_obstacles = obstacles.clone();
            }
        }
        
        // Check if path exists
        let g_start = *self.g_scores.get(&self.s_start).unwrap_or(&i32::MAX);
        if g_start == i32::MAX {
            return None;
        }

        self.reconstruct_path(grid, obstacles)
    }
    
    fn update_environment(&mut self, grid: &Grid, obstacles: &HashSet<Position>) {
        // Only update if obstacles actually changed
        if obstacles != &self.last_known_obstacles {
            self.update_edge_costs_incremental(grid, obstacles);
            self.last_known_obstacles = obstacles.clone();
        }
    }
}


impl DStarLite {
    /// Reconstruct path from start to goal
    fn reconstruct_path(&self, grid: &Grid, obstacles: &HashSet<Position>) -> Option<Vec<Position>> {
        let mut path = vec![self.s_start];
        let mut current = self.s_start;

        // Follow path from start to goal (line 34' from paper)
        while current != self.s_goal {
            // Find best successor: s_start = arg min_{s'∈Succ(s_start)} {c(s_start,s') + g(s')}
            let successors = self.succ(current, grid);
            let next_step = successors
                .iter()
                .filter(|&s_prime| self.c(current, *s_prime, grid, obstacles) != i32::MAX)
                .min_by_key(|&s_prime| {
                    let cost = self.c(current, *s_prime, grid, obstacles);
                    let g_s_prime = *self.g_scores.get(s_prime).unwrap_or(&i32::MAX);
                    if cost == i32::MAX || g_s_prime == i32::MAX {
                        i32::MAX
                    } else {
                        cost.saturating_add(g_s_prime)
                    }
                });

            if let Some(&next) = next_step {
                path.push(next);
                current = next;
                
                // Safety check to prevent infinite loops
                if path.len() > grid.size * grid.size {
                    return None;
                }
            } else {
                // No valid next step found
                return None;
            }
        }

        Some(path)
    }
}

impl DStarLite {
    /// Helper function to compare keys (k1 < k2)
    fn key_less_than(&self, k1: Key, k2: Key) -> bool {
        if k1.k1 != k2.k1 {
            k1.k1 < k2.k1
        } else {
            k1.k2 < k2.k2
        }
    }
    /// EFFICIENT: Update only edges that actually changed
    pub fn update_edge_costs_incremental(&mut self, grid: &Grid, new_obstacles: &HashSet<Position>) {
        let mut changed_vertices = HashSet::new();
        
        // Handle new obstacles
        for &obs_pos in new_obstacles.difference(&self.last_known_obstacles) {
            // Update edges TO this position (now blocked)
            for neighbor in grid.get_neighbors(&obs_pos) {
                self.edge_costs.insert((neighbor, obs_pos), i32::MAX);
                changed_vertices.insert(neighbor);
            }
            changed_vertices.insert(obs_pos);
        }
        
        // Handle removed obstacles
        for &obs_pos in self.last_known_obstacles.difference(new_obstacles) {
            // Update edges TO this position (now passable)
            for neighbor in grid.get_neighbors(&obs_pos) {
                self.edge_costs.insert((neighbor, obs_pos), 1);
                changed_vertices.insert(neighbor);
            }
            changed_vertices.insert(obs_pos);
        }
        
        // Only update vertices that were actually affected
        for &vertex in &changed_vertices {
            self.update_vertex(vertex, grid, new_obstacles);
        }
    }
}
