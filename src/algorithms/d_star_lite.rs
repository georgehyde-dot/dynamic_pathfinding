use crate::algorithms::common::PathfindingAlgorithm;
use crate::grid::{Grid, Position};
use std::collections::{HashMap, HashSet, BinaryHeap};
use std::cmp::Ordering;

/// Represents the priority key for a node in the D* Lite priority queue.
/// We implement Ord in reverse to make the BinaryHeap a min-heap.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Key {
    k1: i32,
    k2: i32,
}

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
/// This is a foundational implementation.
pub struct DStarLite {
    g_scores: HashMap<Position, i32>,
    rhs_scores: HashMap<Position, i32>,
    queue: BinaryHeap<(Key, Position)>,
    k_m: i32, // Key modifier for handling path cost changes
    start: Position,
    goal: Position,
}

impl DStarLite {
    /// Creates a new instance of the D* Lite algorithm.
    pub fn new(start: Position, goal: Position) -> Self {
        let mut d_star = DStarLite {
            g_scores: HashMap::new(),
            rhs_scores: HashMap::new(),
            queue: BinaryHeap::new(),
            k_m: 0,
            start,
            goal,
        };

        // Initialize the algorithm with the goal node.
        d_star.rhs_scores.insert(goal, 0);
        d_star.queue.push((d_star.calculate_key(goal), goal));
        d_star
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

        Key {
            k1: min_score.saturating_add(self.heuristic(pos)).saturating_add(self.k_m),
            k2: min_score,
        }
    }
    
    /// Updates a vertex's rhs-value and its position in the priority queue.
    fn update_vertex(&mut self, pos: Position, grid: &Grid, obstacles: &HashSet<Position>) {
        if pos != self.goal {
            let min_rhs = grid.get_neighbors(&pos)
                .iter()
                .filter(|n| !obstacles.contains(n))
                .map(|n| self.g_scores.get(n).unwrap_or(&i32::MAX).saturating_add(1))
                .min()
                .unwrap_or(i32::MAX);
            self.rhs_scores.insert(pos, min_rhs);
        }

        // Remove the old entry from the queue if it exists.
        self.queue.retain(|(_, p)| *p != pos);

        // Add to the queue if the node is inconsistent (g != rhs).
        if *self.g_scores.get(&pos).unwrap_or(&i32::MAX) != *self.rhs_scores.get(&pos).unwrap_or(&i32::MAX) {
            self.queue.push((self.calculate_key(pos), pos));
        }
    }

    /// Main loop to compute the shortest path by processing the priority queue.
    fn compute_shortest_path(&mut self, grid: &Grid, obstacles: &HashSet<Position>) {
        // Continue as long as the top key is less than the start's key, or the start is inconsistent.
        while !self.queue.is_empty() && 
              (self.queue.peek().unwrap().0 < self.calculate_key(self.start) || 
               *self.rhs_scores.get(&self.start).unwrap_or(&i32::MAX) != *self.g_scores.get(&self.start).unwrap_or(&i32::MAX)) {
            
            let (_, u) = self.queue.pop().unwrap();

            let g_u = *self.g_scores.get(&u).unwrap_or(&i32::MAX);
            let rhs_u = *self.rhs_scores.get(&u).unwrap_or(&i32::MAX);

            if g_u > rhs_u {
                // The node is locally underconsistent. Update g-score and inform predecessors.
                self.g_scores.insert(u, rhs_u);
                for neighbor in grid.get_neighbors(&u) {
                    self.update_vertex(neighbor, grid, obstacles);
                }
            } else {
                // The node is locally overconsistent. Set g-score to infinity and update.
                self.g_scores.insert(u, i32::MAX);
                self.update_vertex(u, grid, obstacles);
                for neighbor in grid.get_neighbors(&u) {
                    self.update_vertex(neighbor, grid, obstacles);
                }
            }
        }
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
        self.start = start;
        self.goal = goal;

        // The core of D* Lite is replanning. In this simulation, we call
        // compute_shortest_path to recalculate based on the agent's current knowledge.
        self.compute_shortest_path(grid, obstacles);
        
        // If the g-score of the start node is infinity, no path exists.
        if self.g_scores.get(&start).unwrap_or(&i32::MAX) == &i32::MAX {
            return None;
        }

        // Reconstruct the path by greedily following the lowest g-scores.
        let mut path = vec![start];
        let mut current = start;

        while current != goal {
            let neighbors = grid.get_neighbors(&current);
            let next_step = neighbors
                .iter()
                .filter(|n| !obstacles.contains(n))
                .min_by_key(|n| self.g_scores.get(n).unwrap_or(&i32::MAX));

            if let Some(&next) = next_step {
                path.push(next);
                current = next;
                // Safety break to prevent infinite loops in case of issues.
                if path.len() > grid.size * grid.size { 
                    return None; 
                }
            } else {
                return None; // Agent is stuck.
            }
        }
        Some(path)
    }
}
