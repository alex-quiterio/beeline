//! The sensor interface: the only way the autonomy stack perceives the world.
//!
//! The simulator implements [`Sensors`]; the autopilot consumes it. Neither depends on the other,
//! so the same autopilot could run against another simulator, a replayed log, or real hardware.

use glam::Vec3;

/// How far the camera can pick out the beacon (m), given a clear line of sight.
pub const CAMERA_RANGE: f32 = 35.0;

/// What a range sensor found.
#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub dist: f32,
    /// Velocity of the point that was hit (a real drone would estimate this by tracking).
    pub vel: Vec3,
    /// Part of the permanent scenery (a real drone would infer this from repeated scans).
    pub fixed: bool,
}

/// State of the payload hook.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Payload {
    None,
    Carrying,
    Delivered,
}

pub trait Sensors {
    /// Range sensor: distance to the first thing along a ray.
    fn raycast(&self, origin: Vec3, dir: Vec3, max_dist: f32) -> Option<Hit>;
    /// Camera: the beacon's position, if it is in view.
    fn spot_beacon(&self, from: Vec3) -> Option<Vec3>;
    /// Payload hook: are we carrying the beacon / has it been delivered?
    fn payload(&self) -> Payload;
}
