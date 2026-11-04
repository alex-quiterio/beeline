//! Bee-inspired optic-flow navigation.
//!
//! Honeybees flying down a corridor (Srinivasan et al., 1991-1996) don't measure distances.
//! They measure *optic flow*: how fast the world slides across each eye, which for a side wall
//! is roughly `speed / distance`. Two simple rules then give robust behaviour:
//!   * Centring: balance left and right flow, which keeps them in the middle of the corridor.
//!   * Speed regulation: hold total flow at a set point, which slows them down where space is tight.
//!
//! The same rules are used on small flying robots (e.g. Franceschini's optic-flow regulator).
//! Here the "eyes" are two side-looking range sensors.

use super::Sensors;
use glam::Vec3;

pub const FLOW_LIMIT: f32 = 3.5; // max total side optic flow (rad/s): the speed regulator's set point
pub const CENTERING_GAIN: f32 = 0.6; // how strongly to balance left/right flow
const EYE_RANGE: f32 = 10.0;
const MAX_CENTERING: f32 = 2.0; // m/s

#[derive(Default, Clone, Copy, Debug)]
pub struct Flow {
    pub left: f32,
    pub right: f32,
}

/// Given a travel direction and a desired speed, return the bee-regulated horizontal velocity
/// and the optic flow measured on each side.
pub fn regulate(sensors: &impl Sensors, pos: Vec3, dir: Vec3, speed: f32) -> (Vec3, Flow) {
    let right_dir = Vec3::new(-dir.z, 0.0, dir.x);
    let eye = |d: Vec3| sensors.raycast(pos, d, EYE_RANGE).map_or(EYE_RANGE, |h| h.dist.max(0.5));
    let (dl, dr) = (eye(-right_dir), eye(right_dir));

    let speed = speed.min(FLOW_LIMIT / (1.0 / dl + 1.0 / dr));
    let flow = Flow { left: speed / dl, right: speed / dr };
    let centring = ((flow.left - flow.right) * CENTERING_GAIN).clamp(-MAX_CENTERING, MAX_CENTERING);
    (dir * speed + right_dir * centring, flow)
}
