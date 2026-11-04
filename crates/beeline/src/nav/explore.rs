//! Frontier exploration (Yamauchi, 1997): to explore an unknown place, repeatedly fly to the
//! nearest *frontier*, the boundary between what you've seen and what you haven't.
//! "Nearest" is measured by a breadth-first search through known free space, so it's the closest
//! frontier you can actually reach, not just the closest in a straight line.

use super::planner::{Cell, Grid};
use glam::Vec2;
use std::collections::VecDeque;

const ESCAPE: i32 = 4; // cells around the start the search may cross regardless of the map

pub fn nearest_frontier(map: &Grid, from: Vec2, min_dist: f32) -> Option<Vec2> {
    let size = map.size();
    let start = map.cell_of(from);
    let mut visited = vec![false; (size * size) as usize];
    let index = |c: Cell| (c.0 >= 0 && c.1 >= 0 && c.0 < size && c.1 < size).then(|| (c.1 * size + c.0) as usize);
    let mut queue = VecDeque::from([start]);
    if let Some(i) = index(start) {
        visited[i] = true;
    }

    while let Some(c) = queue.pop_front() {
        if map.is_frontier(c) && map.center_of(c).distance(from) >= min_dist {
            return Some(map.center_of(c));
        }
        for d in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let n = (c.0 + d.0, c.1 + d.1);
            // Known free space, or anything right around the start: the drone may be sitting
            // inside a wall's safety margin and has to be able to search its way out of it
            let near_start = (n.0 - start.0).abs().max((n.1 - start.1).abs()) <= ESCAPE;
            if let Some(i) = index(n)
                && !visited[i]
                && ((map.is_free(n) && map.is_known(n)) || near_start)
            {
                visited[i] = true;
                queue.push_back(n);
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::vec2;

    #[test]
    fn finds_the_edge_of_the_known_world() {
        let mut map = Grid::new(20.0, 1.0);
        map.mark_seen(vec2(0.0, 0.0), vec2(12.0, 0.0)); // a corridor we have looked down
        map.mark_wall(vec2(0.0, 1.5), 1.8); // ...and the drone is inside a wall's safety margin
        let f = nearest_frontier(&map, vec2(0.0, 0.0), 5.0).expect("a frontier");
        assert!(f.distance(vec2(0.0, 0.0)) >= 5.0);
        assert!(map.is_frontier_at(f));
    }

    #[test]
    fn nothing_left_when_everything_is_known() {
        let map = Grid::from_footprints(10.0, 1.0, &[], 0.0);
        assert!(nearest_frontier(&map, vec2(0.0, 0.0), 1.0).is_none());
    }
}
