//! Global path planner: A* over a 2D occupancy grid of the city, then string-pulling
//! to turn the jagged grid path into a few straight segments.

use glam::{vec2, Vec2};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

type Cell = (i32, i32);
const NEIGHBOURS: [Cell; 8] = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)];

pub struct Grid {
    origin: Vec2,
    cell: f32,
    size: i32,
    blocked: Vec<bool>,
}

impl Grid {
    pub fn new(half_extent: f32, cell: f32) -> Self {
        let size = (half_extent * 2.0 / cell).ceil() as i32;
        Self { origin: Vec2::splat(-half_extent), cell, size, blocked: vec![false; (size * size) as usize] }
    }

    /// Build a grid from rectangular footprints `(centre, half_size)`, each grown by `inflate`.
    pub fn from_footprints(half_extent: f32, cell: f32, footprints: &[(Vec2, Vec2)], inflate: f32) -> Self {
        let mut grid = Self::new(half_extent, cell);
        for &(c, h) in footprints {
            grid.block_rect(c, h, inflate);
        }
        grid
    }

    /// Mark a rectangle as blocked, grown by `inflate` so paths keep a safety margin.
    pub fn block_rect(&mut self, center: Vec2, half: Vec2, inflate: f32) {
        let (a, b) = (self.cell_of(center - half - inflate), self.cell_of(center + half + inflate));
        for y in a.1.max(0)..=b.1.min(self.size - 1) {
            for x in a.0.max(0)..=b.0.min(self.size - 1) {
                self.blocked[(y * self.size + x) as usize] = true;
            }
        }
    }

    pub fn line_of_sight(&self, a: Vec2, b: Vec2) -> bool {
        let steps = ((b - a).length() / (self.cell * 0.5)).ceil().max(1.0) as i32;
        (0..=steps).all(|i| self.is_free(self.cell_of(a.lerp(b, i as f32 / steps as f32))))
    }

    /// Shortest collision-free path from `start` to `goal`, as a list of corner points.
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
            if c == g { break; }
            if f - h(c) > cost[i] { continue; } // stale queue entry
            for (dx, dy) in NEIGHBOURS {
                let nb = (c.0 + dx, c.1 + dy);
                let diagonal = dx != 0 && dy != 0;
                if !self.is_free(nb) || (diagonal && !(self.is_free((c.0 + dx, c.1)) && self.is_free((c.0, c.1 + dy)))) {
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
        if cost[idx(g)] == u32::MAX { return None; }

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
            while j > i && !self.line_of_sight(cur, cells[j]) { j -= 1; }
            path.push(cells[j]);
            i = j + 1;
        }
        *path.last_mut().unwrap() = goal;
        Some(path)
    }

    fn cell_of(&self, p: Vec2) -> Cell {
        let c = ((p - self.origin) / self.cell).floor();
        (c.x as i32, c.y as i32)
    }

    fn center_of(&self, c: Cell) -> Vec2 {
        self.origin + (vec2(c.0 as f32, c.1 as f32) + 0.5) * self.cell
    }

    fn is_free(&self, c: Cell) -> bool {
        c.0 >= 0 && c.1 >= 0 && c.0 < self.size && c.1 < self.size && !self.blocked[(c.1 * self.size + c.0) as usize]
    }

    /// The closest free cell (searching outward in square rings), for when we start inside a margin.
    fn nearest_free(&self, c: Cell) -> Option<Cell> {
        (0..15).find_map(|r: i32| {
            (-r..=r).flat_map(|dy| (-r..=r).map(move |dx| (dx, dy)))
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
        for (c, h) in [(vec2(0.0, 5.0), vec2(6.0, 1.0)), (vec2(0.0, -5.0), vec2(6.0, 1.0)),
                       (vec2(5.0, 0.0), vec2(1.0, 6.0)), (vec2(-5.0, 0.0), vec2(1.0, 6.0))] {
            grid.block_rect(c, h, 0.0);
        }
        assert!(grid.find_path(vec2(0.0, 0.0), vec2(8.5, 8.5)).is_none());
    }
}
