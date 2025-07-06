use crate::algorithms::common::PathfindingAlgorithm;
use crate::grid::{Grid, Position, Cell};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Represents the priority key for a node in the D* Lite priority queue.
/// The priority queue is a min-heap, so we implement `Ord` in reverse.
#[derive(Clone, Copy, PartialEq)]
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
        if self.k1 != other.k1 {
            other.k1.cmp(&self.k1)
        } else {
            other.k2.cmp(&self.k2)
        }
    }
}

/// Implements the D* Lite pathfinding algorithm based on the 2002 paper by S. Koenig and M. Likhachev.
pub struct DStarLite {
    g_scores: HashMap<Position, i32>,
    rhs_scores: HashMap<Position, i32>,
    queue: BinaryHeap<(Key, Position)>,
    k_m: i32, // Key modifier for handling path cost changes as the agent moves.
    last_start: Position,
    goal: Position,
}

impl DStarLite {
    /// Creates a new instance of the D* Lite algorithm.
    /// Corresponds to the `Initialize()` procedure in the paper.
    pub fn new(start: Position, goal: Position) -> Self {
        let mut d_star = DStarLite {
            g_scores: HashMap::new(),
            rhs_scores: HashMap::new(),
            queue: BinaryHeap::new(),
            k_m: 0, // (line 03')
            last_start: start,
            goal,
        };

        // (line 04') All g and rhs values are implicitly infinity.
        // (line 05')
        d_star.rhs_scores.insert(goal, 0);
        
        // (line 06')
        let key = d_star.calculate_key(start, goal);
        d_star.queue.push((key, goal));
        d_star
    }

    /// Calculates the heuristic value (Manhattan distance) from a position to the start.
    fn heuristic(&self, from: Position, to: Position) -> i32 {
        ((from.x as i32 - to.x as i32).abs() + (from.y as i32 - to.y as i32).abs())
    }

    /// Calculates the key for a given position for the priority queue.
    /// Corresponds to `CalculateKey(s)` in the paper.
    fn calculate_key(&self, start: Position, pos: Position) -> Key {
        let g = *self.g_scores.get(&pos).unwrap_or(&i32::MAX);
        let rhs = *self.rhs_scores.get(&pos).unwrap_or(&i32::MAX);
        let min_score = g.min(rhs);

        // (line 01')
        Key {
            k1: min_score.saturating_add(self.heuristic(pos, start)).saturating_add(self.k_m),
            k2: min_score,
        }
    }
    
    /// Updates a vertex's rhs-value and its position in the priority queue.
    /// Corresponds to `UpdateVertex(u)` in the paper.
    fn update_vertex(&mut self, u: Position, start: Position, grid: &Grid, obstacles: &HashSet<Position>) {
        // (line 07')
        if u != self.goal {
            let min_rhs = grid
                .get_neighbors(&u) // These are the successors of u
                .iter()
                .filter(|s_prime| grid.cells[s_prime.x][s_prime.y] != Cell::Wall && !obstacles.contains(s_prime))
                .map(|s_prime| self.g_scores.get(s_prime).unwrap_or(&i32::MAX).saturating_add(1)) // c(u, s') + g(s')
                .min()
                .unwrap_or(i32::MAX);
            self.rhs_scores.insert(u, min_rhs);
        }

        // (line 08')
        self.queue.retain(|(_, p)| *p != u);

        // (line 09')
        if *self.g_scores.get(&u).unwrap_or(&i32::MAX) != *self.rhs_scores.get(&u).unwrap_or(&i32::MAX) {
            self.queue.push((self.calculate_key(start, u), u));
        }
    }

    /// Main loop to compute the shortest path by processing the priority queue.
    /// Corresponds to `ComputeShortestPath()` in the paper.
    fn compute_shortest_path(&mut self, start: Position, grid: &Grid, obstacles: &HashSet<Position>) {
        // (line 10')
        while !self.queue.is_empty() {
            let top_key = self.queue.peek().unwrap().0;
            let start_key = self.calculate_key(start, start);
            let start_rhs = *self.rhs_scores.get(&start).unwrap_or(&i32::MAX);
            let start_g = *self.g_scores.get(&start).unwrap_or(&i32::MAX);

            if top_key >= start_key && start_rhs == start_g {
                break;
            }

            // (line 12')
            let (_, u) = self.queue.pop().unwrap();
            
            // (line 13' - 14') Logic to handle outdated keys due to k_m changes
            let k_old = top_key;
            let k_new = self.calculate_key(start, u);
            if k_old < k_new {
                self.queue.push((k_new, u));
                continue;
            }

            let g_u = *self.g_scores.get(&u).unwrap_or(&i32::MAX);
            let rhs_u = *self.rhs_scores.get(&u).unwrap_or(&i32::MAX);

            // (line 15')
            if g_u > rhs_u {
                // Node is underconsistent, make it consistent.
                // (line 16')
                self.g_scores.insert(u, rhs_u);
                // (line 17') Propagate changes to predecessors.
                for s in grid.get_neighbors(&u) {
                    self.update_vertex(s, start, grid, obstacles);
                }
            } else {
                // (line 18') Node is overconsistent.
                // (line 19')
                self.g_scores.insert(u, i32::MAX);
                // (line 20') Propagate changes to predecessors and self.
                self.update_vertex(u, start, grid, obstacles);
                for s in grid.get_neighbors(&u) {
                    self.update_vertex(s, start, grid, obstacles);
                }
            }
        }
    }
}

impl PathfindingAlgorithm for DStarLite {
    /// Finds a path from start to goal using the D* Lite algorithm.
    /// This corresponds to the `Main()` procedure in the paper.
    fn find_path(
        &mut self,
        grid: &Grid,
        start: Position,
        goal: Position,
        obstacles: &HashSet<Position>,
    ) -> Option<Vec<Position>> {
        // (line 29', 31') The agent moves, updating the start position.
        let last = self.last_start;
        self.last_start = start;
        
        // (line 28', 29') Scan for changed edge costs. In our simulation, this is handled
        // by the agent discovering new obstacles.
        // (line 30') Update k_m due to agent movement.
        self.k_m = self.k_m.saturating_add(self.heuristic(last, start));

        // Here, a full implementation would iterate through only the *changed* edges.
        // For simplicity in this simulation, we can re-evaluate the world based on
        // the agent's current knowledge.

        // (line 35')
        self.compute_shortest_path(start, grid, obstacles);
        
        // (line 25') Check if a path exists.
        if self.g_scores.get(&start).unwrap_or(&i32::MAX) == &i32::MAX {
            return None;
        }

        // Reconstruct the path by greedily following the lowest g-scores.
        let mut path = vec![start];
        let mut current = start;

        // (line 24') Loop until goal is reached.
        while current != goal {
            // (line 26') Move to successor s' that minimizes c(s, s') + g(s').
            let next_step = grid
                .get_neighbors(&current)
                .iter()
                .filter(|n| grid.cells[n.x][n.y] != Cell::Wall && !obstacles.contains(n))
                .min_by_key(|n| self.g_scores.get(n).unwrap_or(&i32::MAX).saturating_add(1));

            if let Some(&next) = next_step {
                path.push(next);
                current = next;
                // Safety break to prevent infinite loops.
                if path.len() > grid.size * grid.size { return None; }
            } else {
                return None; // Agent is stuck.
            }
        }
        Some(path)
    }
}
