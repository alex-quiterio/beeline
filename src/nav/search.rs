//! Searching for a target that moves.
//!
//! Frontier exploration alone assumes that once a place has been seen, it's done. A moving target
//! breaks that: it can wander back into streets the drone already checked. Search-and-rescue
//! planners handle this by letting certainty decay: the longer since you last looked somewhere,
//! the likelier a moving target has wandered back in.
//!
//! The search map keeps, on a coarse grid, **when the camera last had a clear view** of each spot.
//! Candidate goals are then scored by `time since seen / (distance + d0)`:
//!   * stale streets (known, open, not looked at recently), and
//!   * the nearest frontier (never seen at all: maximum staleness).

use super::planner::Grid;
use crate::sensors::CAMERA_RANGE;
use glam::{Vec2, Vec3, vec2};

const CELL: f32 = 4.0; // m
const MAX_AGE: f32 = 300.0; // s; staleness is capped here (and frontiers count as this)
const MIN_AGE: f32 = 30.0; // s; anything seen more recently isn't worth revisiting yet
const DIST0: f32 = 30.0; // m; distance offset in the score, so nearby isn't infinitely attractive
const OBSERVE_EVERY: f32 = 0.5; // s

pub struct SearchMap {
    origin: Vec2,
    size: i32,
    last_seen: Vec<f32>,
    timer: f32,
}

impl SearchMap {
    pub fn new(half_extent: f32) -> Self {
        let size = (half_extent * 2.0 / CELL).ceil() as i32;
        Self {
            origin: Vec2::splat(-half_extent),
            size,
            last_seen: vec![f32::NEG_INFINITY; (size * size) as usize],
            timer: 0.0,
        }
    }

    /// Mark every spot the camera can currently see (in range, clear line of sight on the map).
    pub fn observe(&mut self, dt: f32, map: &Grid, pos: Vec3, time: f32) {
        self.timer -= dt;
        if self.timer > 0.0 {
            return;
        }
        self.timer = OBSERVE_EVERY;
        let p = vec2(pos.x, pos.z);
        let r = (CAMERA_RANGE / CELL).ceil() as i32;
        let c = self.cell_of(p);
        for y in (c.1 - r).max(0)..=(c.1 + r).min(self.size - 1) {
            for x in (c.0 - r).max(0)..=(c.0 + r).min(self.size - 1) {
                let center = self.center_of((x, y));
                if center.distance(p) <= CAMERA_RANGE && map.line_of_sight(p, center) {
                    self.last_seen[(y * self.size + x) as usize] = time;
                }
            }
        }
    }

    /// Seconds since the camera last saw the spot `p` (capped).
    pub fn age(&self, p: Vec2, time: f32) -> f32 {
        let c = self.cell_of(p);
        if c.0 < 0 || c.1 < 0 || c.0 >= self.size || c.1 >= self.size {
            return 0.0;
        }
        (time - self.last_seen[(c.1 * self.size + c.0) as usize]).min(MAX_AGE)
    }

    /// The best place to look next: a stale street or the given frontier, whichever scores higher.
    pub fn best_goal(&self, map: &Grid, from: Vec2, time: f32, frontier: Option<Vec2>) -> Option<Vec2> {
        let score = |age: f32, p: Vec2| age / (p.distance(from) + DIST0);
        let mut best = frontier.map(|f| (score(MAX_AGE, f), f));
        for y in 0..self.size {
            for x in 0..self.size {
                let age = (time - self.last_seen[(y * self.size + x) as usize]).min(MAX_AGE);
                let center = self.center_of((x, y));
                let cell = map.cell_of(center);
                // Only open ground we know about (never building interiors we can't see into)
                if age < MIN_AGE || age == MAX_AGE && !map.is_known(cell) || !map.is_free(cell) || !map.is_known(cell) {
                    continue;
                }
                let s = score(age, center);
                if best.is_none_or(|(b, _)| s > b) {
                    best = Some((s, center));
                }
            }
        }
        best.map(|(_, p)| p)
    }

    fn cell_of(&self, p: Vec2) -> (i32, i32) {
        let c = ((p - self.origin) / CELL).floor();
        (c.x as i32, c.y as i32)
    }

    fn center_of(&self, c: (i32, i32)) -> Vec2 {
        self.origin + (vec2(c.0 as f32, c.1 as f32) + 0.5) * CELL
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_streets_become_worth_revisiting() {
        // Open and fully known, and small enough that the camera sees all of it from the centre
        let map = Grid::from_footprints(20.0, 1.0, &[], 0.0);
        let mut search = SearchMap::new(20.0);
        search.observe(1.0, &map, Vec3::ZERO, 0.0); // look around at t=0
        let here = vec2(0.0, 0.0);
        assert!(search.best_goal(&map, here, 10.0, None).is_none(), "just looked: nothing is stale");
        assert!(search.best_goal(&map, here, 120.0, None).is_some(), "two minutes later it may have moved back");
        assert!(search.age(here, 120.0) >= 119.0);
    }
}
