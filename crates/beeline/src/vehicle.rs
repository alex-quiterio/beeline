//! The vehicle: the airframe's physical properties and the two interfaces every layer agrees on,
//! what goes *into* the aircraft ([`Inputs`]) and what is known *about* it ([`State`]).
//!
//! Both the physics (`sim`) and the controller (`flight`) depend on this module, never on each
//! other, so either can be replaced (a different simulator, a real airframe) independently.

use glam::Vec3;

/// Gravity (m/s²).
pub const G: f32 = 9.81;
/// Tilt at full pitch/roll stick (rad).
pub const MAX_TILT: f32 = 0.55;
/// Throttle = 1 adds this fraction of hover thrust.
pub const THRUST_RANGE: f32 = 0.7;
/// Yaw rate at full yaw stick (rad/s).
pub const MAX_YAW_RATE: f32 = 2.0;
/// Linear aerodynamic drag coefficient (1/s).
pub const DRAG: f32 = 0.35;

/// Stick inputs, each in [-1, 1].
#[derive(Default, Clone, Copy, Debug)]
pub struct Inputs {
    pub throttle: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

/// What is known about the vehicle (an estimator's output in a real stack).
#[derive(Clone, Copy, Debug)]
pub struct State {
    pub pos: Vec3,
    pub vel: Vec3,
    pub yaw: f32,
}
