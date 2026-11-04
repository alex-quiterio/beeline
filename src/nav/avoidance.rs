//! Reactive collision avoidance from a 3D ring of range sensors (a simple spinning lidar).
//!
//! * Collision prevention (as in PX4): remove the part of the commanded velocity that points at a
//!   nearby obstacle, more strongly the closer it is. The drone slides along walls instead of
//!   bouncing off them, and never accelerates into something it can see.
//! * Time-to-collision: for moving obstacles on a collision course, sidestep perpendicular to their
//!   motion (or hold back, for things moving vertically), with urgency growing as impact nears.

use super::Sensors;
use glam::Vec3;
use std::f32::consts::TAU;

pub const RAYS: usize = 24; // per ring, evenly spaced around the drone
pub const ELEVATIONS: [f32; 3] = [-0.35, 0.0, 0.35]; // rings at -20°, 0°, +20° (rad)
pub const SENSOR_RANGE: f32 = 15.0;
pub const SLOW_R: f32 = 5.0; // start removing velocity toward an obstacle at this distance
const STOP_R: f32 = 1.5; // fully stop approaching (and back off) inside this distance
const TTC_HORIZON: f32 = 3.0; // react to moving obstacles that would hit within this many seconds

/// One sensor reading of the level ring, kept for debug drawing.
#[derive(Clone, Copy, Debug)]
pub struct SensorRay {
    pub from: Vec3,
    pub to: Vec3,
    pub close: bool,
}

pub struct Avoidance {
    pub rays: Vec<SensorRay>,
    pub active: bool,
}

/// Adjust a desired velocity `v` so it is safe given what the sensors see.
///
/// Two passes: first dodge moving obstacles, then apply collision prevention as a final *safety
/// filter*, so no other behaviour (dodging included) can command the drone into a wall.
pub fn avoid(sensors: &impl Sensors, pos: Vec3, drone_vel: Vec3, mut v: Vec3) -> (Vec3, Avoidance) {
    let mut out = Avoidance { rays: Vec::with_capacity(RAYS), active: false };

    // Scan: (horizontal bearing, 3D direction, hit)
    let mut hits = Vec::new();
    for elevation in ELEVATIONS {
        for i in 0..RAYS {
            let a = i as f32 / RAYS as f32 * TAU;
            let flat = Vec3::new(a.cos(), 0.0, a.sin());
            let dir = flat * elevation.cos() + Vec3::Y * elevation.sin();
            let hit = sensors.raycast(pos, dir, SENSOR_RANGE);
            if elevation == 0.0 {
                let dist = hit.as_ref().map_or(SENSOR_RANGE, |h| h.dist);
                out.rays.push(SensorRay { from: pos, to: pos + dir * dist, close: dist < SLOW_R });
            }
            if let Some(h) = hit {
                hits.push((flat, dir, h.dist * elevation.cos(), h));
            }
        }
    }

    // Pass 1: moving obstacles on a collision course (3D closing speed) -> sidestep / hold back
    for (flat, dir, _, h) in &hits {
        if h.vel.length() < 0.5 { continue; }
        let closing = (drone_vel - h.vel).dot(*dir);
        let ttc = if closing > 0.1 { h.dist / closing } else { f32::INFINITY };
        if ttc < TTC_HORIZON {
            let urgency = 1.0 - ttc / TTC_HORIZON;
            let mut side = Vec3::new(-h.vel.z, 0.0, h.vel.x).normalize_or_zero();
            if side.dot(*flat) > 0.0 {
                side = -side; // the side away from the obstacle
            }
            v += (side * 5.0 - *flat * 3.0) * urgency / ELEVATIONS.len() as f32;
            out.active = true;
        }
    }

    // Pass 2 (safety filter): remove velocity toward anything close, relative to its own motion
    let mut back_off = Vec3::ZERO;
    for (flat, _, flat_dist, h) in &hits {
        let closing = (v - h.vel).dot(*flat);
        if closing > 0.0 && *flat_dist < SLOW_R {
            let w = ((SLOW_R - flat_dist) / (SLOW_R - STOP_R)).clamp(0.0, 1.0);
            v -= *flat * closing * w;
            out.active |= w > 0.3;
        }
        if *flat_dist < STOP_R {
            back_off -= *flat * (STOP_R - flat_dist); // too close: back off
        }
    }
    (v + back_off.clamp_length_max(1.0) * 3.0, out)
}
