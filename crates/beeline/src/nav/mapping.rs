//! Mapping: turn lidar scans into the drone's own occupancy map.
//!
//! Each scan casts a fan of horizontal rays. Everything a ray passes through is now *known* to be
//! empty; where it hits a **fixed** object, a wall (plus safety margin) is recorded. Moving things
//! (gates, lifts, patrols) are left out of the map; reactive avoidance handles those.

use super::Sensors;
use super::planner::Grid;
use glam::{Vec3, vec2};
use std::f32::consts::TAU;

pub const SCAN_RAYS: usize = 180; // 2° apart: fine enough to leave no gaps at 25 m on a 1 m grid
pub const SCAN_RANGE: f32 = 25.0;
pub const MARGIN: f32 = 1.8; // safety margin around walls (m)

/// One lidar sweep: where it was taken and the range of each ray (None = nothing within range).
/// Ray `i` points at angle `i / SCAN_RAYS * 2π` in the simulator's horizontal plane.
#[derive(Clone, Debug)]
pub struct Scan {
    pub time: f32,
    pub pos: Vec3,
    pub ranges: Vec<Option<f32>>,
}

pub fn integrate_scan(map: &mut Grid, sensors: &impl Sensors, pos: Vec3, time: f32) -> Scan {
    let origin = vec2(pos.x, pos.z);
    let mut ranges = Vec::with_capacity(SCAN_RAYS);
    for i in 0..SCAN_RAYS {
        let a = i as f32 / SCAN_RAYS as f32 * TAU;
        let dir = vec2(a.cos(), a.sin());
        let hit = sensors.raycast(pos, Vec3::new(dir.x, 0.0, dir.y), SCAN_RANGE);
        ranges.push(hit.map(|h| h.dist));
        let end = origin + dir * hit.map_or(SCAN_RANGE, |h| h.dist);
        map.mark_seen(origin, end);
        if let Some(h) = hit
            && h.fixed
        {
            map.mark_wall(end + dir * 0.3, MARGIN); // nudge inside the surface
        }
    }
    Scan { time, pos, ranges }
}
