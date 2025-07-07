pub fn astar<N, C, FN, IN, FH, FS>(
    start: &N,
    mut successors: FN,
    mut heuristic: FH,
    mut success: FS,
) -> Option<(Vec<N>, C)>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
    FN: FnMut(&N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let mut to_see = BinaryHeap::new();
    to_see.push(SmallestCostHolder {
        estimated_cost: Zero::zero(),
        cost: Zero::zero(),
        index: 0,
    });
    let mut parents: FxIndexMap<N, (usize, C)> = FxIndexMap::default();
    parents.insert(start.clone(), (usize::MAX, Zero::zero()));
    while let Some(SmallestCostHolder { cost, index, .. }) = to_see.pop() {
        let successors = {
            let (node, &(_, c)) = parents.get_index(index).unwrap(); // Cannot fail
            if success(node) {
                let path = reverse_path(&parents, |&(p, _)| p, index);
                return Some((path, cost));
            }
            // We may have inserted a node several time into the binary heap if we found
            // a better way to access it. Ensure that we are currently dealing with the
            // best path and discard the others.
            if cost > c {
                continue;
            }
            successors(node)
        };
        for (successor, move_cost) in successors {
            let new_cost = cost + move_cost;
            let h; // heuristic(&successor)
            let n; // index for successor
            match parents.entry(successor) {
                Vacant(e) => {
                    h = heuristic(e.key());
                    n = e.index();
                    e.insert((index, new_cost));
                }
                Occupied(mut e) => {
                    if e.get().1 > new_cost {
                        h = heuristic(e.key());
                        n = e.index();
                        e.insert((index, new_cost));
                    } else {
                        continue;
                    }
                }
            }

            to_see.push(SmallestCostHolder {
                estimated_cost: new_cost + h,
                cost: new_cost,
                index: n,
            });
        }
    }
    None
}