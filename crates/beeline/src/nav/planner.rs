//! The drone's map and its path planner.
//!
//! The map is a 2D occupancy grid with a *knowledge* layer: every cell starts **unknown** and is
//! learned from the lidar as the drone flies (see `mapping.rs`). Walls are inflated by a safety
//! margin. A* treats unknown cells as free ("optimistic planning") and the route is replanned as
//! the map fills in. String pulling turns the jagged grid path into a few straight segments.

use glam::{Vec2, vec2};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub type Cell = (i32, i32);
const NEIGHBOURS: [Cell; 8] = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)];
const SIDES: [Cell; 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CellState {
    Unknown,
    Free,
    /// Within the safety margin of a wall
    Margin,
    Wall,
}

pub struct Grid {
    origin: Vec2,
    cell: f32,
    size: i32,
    known: Vec<bool>,
    blocked: Vec<bool>, // wall or margin
    wall: Vec<bool>,
    known_count: usize,
}

impl Grid {
    /// A grid where everything is still unknown.
    pub fn new(half_extent: f32, cell: f32) -> Self {
        let size = (half_extent * 2.0 / cell).ceil() as i32;
        let n = (size * size) as usize;
        Self {
            origin: Vec2::splat(-half_extent),
            cell,
            size,
            known: vec![false; n],
            blocked: vec![false; n],
            wall: vec![false; n],
            known_count: 0,
        }
    }

    /// A fully known grid from rectangular footprints `(centre, half_size)`, each grown by `inflate`.
    pub fn from_footprints(half_extent: f32, cell: f32, footprints: &[(Vec2, Vec2)], inflate: f32) -> Self {
        let mut grid = Self::new(half_extent, cell);
        grid.known.fill(true);
        grid.known_count = grid.known.len();
        for &(c, h) in footprints {
            grid.block_rect(c, h, inflate);
        }
        grid
    }

    /// Mark a rectangle as wall, grown by an `inflate` margin.
    pub fn block_rect(&mut self, center: Vec2, half: Vec2, inflate: f32) {
        let (a, b) = (self.cell_of(center - half - inflate), self.cell_of(center + half + inflate));
        for y in a.1..=b.1 {
            for x in a.0..=b.0 {
                if let Some(i) = self.index((x, y)) {
                    self.blocked[i] = true;
                    self.learn(i);
                }
            }
        }
    }

    /// The straight line from `a` to `b` (excluding its end) has been seen to be empty.
    pub fn mark_seen(&mut self, a: Vec2, b: Vec2) {
        let steps = ((b - a).length() / (self.cell * 0.5)) as i32;
        for s in 0..steps {
            if let Some(i) = self.index(self.cell_of(a.lerp(b, s as f32 / steps as f32))) {
                self.learn(i);
            }
        }
    }

    /// This cell has been observed (e.g. by another drone).
    pub fn mark_known(&mut self, c: Cell) {
        if let Some(i) = self.index(c) {
            self.learn(i);
        }
    }

    /// A wall was seen at `p`: block it and every cell within `inflate` of it.
    pub fn mark_wall(&mut self, p: Vec2, inflate: f32) {
        let c = self.cell_of(p);
        let Some(i) = self.index(c) else { return };
        if self.wall[i] {
            return;
        }
        self.wall[i] = true;
        let r = (inflate / self.cell).ceil() as i32;
        for dy in -r..=r {
            for dx in -r..=r {
                if ((dx * dx + dy * dy) as f32).sqrt() * self.cell <= inflate + 1e-3
                    && let Some(j) = self.index((c.0 + dx, c.1 + dy))
                {
                    self.blocked[j] = true;
                    self.learn(j);
                }
            }
        }
    }

    pub fn state(&self, c: Cell) -> CellState {
        match self.index(c) {
            None => CellState::Unknown,
            Some(i) if self.wall[i] => CellState::Wall,
            Some(i) if self.blocked[i] => CellState::Margin,
            Some(i) if self.known[i] => CellState::Free,
            Some(_) => CellState::Unknown,
        }
    }

    pub fn size(&self) -> i32 {
        self.size
    }

    /// Fraction of the map that has been seen.
    pub fn explored(&self) -> f32 {
        self.known_count as f32 / self.known.len() as f32
    }

    /// Map a world position to 0..1 across the grid (for drawing a minimap).
    pub fn to_unit(&self, p: Vec2) -> Vec2 {
        (p - self.origin) / (self.cell * self.size as f32)
    }

    /// A frontier cell is known, free, and touches the unknown: the edge of what has been explored.
    pub fn is_frontier(&self, c: Cell) -> bool {
        self.is_free(c)
            && self.is_known(c)
            && SIDES.iter().any(|d| self.index((c.0 + d.0, c.1 + d.1)).is_some_and(|j| !self.known[j]))
    }

    pub fn is_frontier_at(&self, p: Vec2) -> bool {
        self.is_frontier(self.cell_of(p))
    }

    pub fn is_known(&self, c: Cell) -> bool {
        self.index(c).is_some_and(|i| self.known[i])
    }

    /// Not blocked (unknown cells count as free) and inside the map.
    pub fn is_free(&self, c: Cell) -> bool {
        self.index(c).is_some_and(|i| !self.blocked[i])
    }

    pub fn cell_of(&self, p: Vec2) -> Cell {
        let c = ((p - self.origin) / self.cell).floor();
        (c.x as i32, c.y as i32)
    }

    pub fn center_of(&self, c: Cell) -> Vec2 {
        self.origin + (vec2(c.0 as f32, c.1 as f32) + 0.5) * self.cell
    }

    pub fn line_of_sight(&self, a: Vec2, b: Vec2) -> bool {
        let steps = ((b - a).length() / (self.cell * 0.5)).ceil().max(1.0) as i32;
        (0..=steps).all(|i| self.is_free(self.cell_of(a.lerp(b, i as f32 / steps as f32))))
    }

    /// Shortest path from `start` to `goal` through cells not known to be blocked.
    pub fn find_path(&self, start: Vec2, goal: Vec2) -> Option<Vec<Vec2>> {
        let s = self.nearest_free(self.cell_of(start))?;
        let g = self.nearest_free(self.cell_of(goal))?;
        let idx = |c: Cell| (c.1 * self.size + c.0) as usize;
        let cell_at = |i: usize| ((i as i32) % self.size, (i as i32) / self.size);
        let h = |c: Cell| {
            let (dx, dy) = ((c.0 - g.0).unsigned_abs(), (c.1 - g.1).unsigned_abs());
            10 * dx.max(dy) + 4 * dx.min(dy) // octile distance, x10 to stay in integers
        };

        let n = (self.size * self.size) as usize;
        let (mut cost, mut parent) = (vec![u32::MAX; n], vec![usize::MAX; n]);
        let mut open = BinaryHeap::new();
        cost[idx(s)] = 0;
        open.push(Reverse((h(s), idx(s))));

        while let Some(Reverse((f, i))) = open.pop() {
            let c = cell_at(i);
            if c == g {
                break;
            }
            if f - h(c) > cost[i] {
                continue;
            } // stale queue entry
            for (dx, dy) in NEIGHBOURS {
                let nb = (c.0 + dx, c.1 + dy);
                let diagonal = dx != 0 && dy != 0;
                if !self.is_free(nb) || (diagonal && !(self.is_free((c.0 + dx, c.1)) && self.is_free((c.0, c.1 + dy))))
                {
                    continue; // blocked, or a diagonal that would cut a corner
                }
                let (j, new_cost) = (idx(nb), cost[i] + if diagonal { 14 } else { 10 });
                if new_cost < cost[j] {
                    cost[j] = new_cost;
                    parent[j] = i;
                    open.push(Reverse((new_cost + h(nb), j)));
                }
            }
        }
        if cost[idx(g)] == u32::MAX {
            return None;
        }

        let mut cells = vec![];
        let mut i = idx(g);
        while i != usize::MAX {
            cells.push(self.center_of(cell_at(i)));
            i = parent[i];
        }
        cells.reverse();

        // String pulling: from each point, jump to the furthest cell still in straight line of sight
        let mut path = vec![start];
        let mut i = 0;
        while i < cells.len() {
            let cur = *path.last().unwrap();
            let mut j = cells.len() - 1;
            while j > i && !self.line_of_sight(cur, cells[j]) {
                j -= 1;
            }
            path.push(cells[j]);
            i = j + 1;
        }
        *path.last_mut().unwrap() = goal;
        Some(path)
    }

    fn index(&self, c: Cell) -> Option<usize> {
        (c.0 >= 0 && c.1 >= 0 && c.0 < self.size && c.1 < self.size).then(|| (c.1 * self.size + c.0) as usize)
    }

    fn learn(&mut self, i: usize) {
        if !self.known[i] {
            self.known[i] = true;
            self.known_count += 1;
        }
    }

    /// The closest free cell (searching outward in square rings), for when we start inside a margin.
    fn nearest_free(&self, c: Cell) -> Option<Cell> {
        (0..15).find_map(|r: i32| {
            (-r..=r)
                .flat_map(|dy| (-r..=r).map(move |dx| (dx, dy)))
                .filter(|(dx, dy)| dx.abs().max(dy.abs()) == r)
                .map(|(dx, dy)| (c.0 + dx, c.1 + dy))
                .find(|&n| self.is_free(n))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn routes_around_a_wall() {
        // A wall between start and goal, with a gap only at the top
        let grid = Grid::from_footprints(20.0, 1.0, &[(vec2(0.0, -4.0), vec2(1.0, 14.0))], 0.5);
        let (start, goal) = (vec2(-10.0, -5.0), vec2(10.0, -5.0));
        assert!(!grid.line_of_sight(start, goal));
        let path = grid.find_path(start, goal).expect("a path exists");
        assert_eq!((path[0], *path.last().unwrap()), (start, goal));
        assert!(path.windows(2).all(|w| grid.line_of_sight(w[0], w[1])), "every segment is collision free");
        assert!(path.len() <= 5, "string pulling keeps only the corners: {path:?}");
    }

    #[test]
    fn no_path_when_enclosed() {
        let mut grid = Grid::new(10.0, 1.0);
        for (c, h) in [
            (vec2(0.0, 5.0), vec2(6.0, 1.0)),
            (vec2(0.0, -5.0), vec2(6.0, 1.0)),
            (vec2(5.0, 0.0), vec2(1.0, 6.0)),
            (vec2(-5.0, 0.0), vec2(1.0, 6.0)),
        ] {
            grid.block_rect(c, h, 0.0);
        }
        assert!(grid.find_path(vec2(0.0, 0.0), vec2(8.5, 8.5)).is_none());
    }

    #[test]
    fn learning_walls_and_frontiers() {
        let mut grid = Grid::new(10.0, 1.0);
        assert_eq!(grid.explored(), 0.0);
        grid.mark_seen(vec2(0.0, 0.0), vec2(5.0, 0.0));
        grid.mark_wall(vec2(5.2, 0.0), 1.0);
        assert_eq!(grid.state(grid.cell_of(vec2(5.2, 0.0))), CellState::Wall);
        assert_eq!(grid.state(grid.cell_of(vec2(1.0, 0.0))), CellState::Free);
        assert!(grid.is_frontier_at(vec2(1.0, 0.0)), "seen cells next to unseen ones are frontier");
        assert!(grid.explored() > 0.0);
    }
}
