//! Flight controller: turns a velocity setpoint into stick inputs.
//!
//! Cascade (as in PX4 / ArduPilot):
//!   velocity setpoint -> rate limiter -> PI controller + drag feedforward -> desired acceleration
//!   -> tilt angles / throttle -> sticks.
//! The integral term learns steady disturbances such as wind, so the drone holds position in a breeze.

use glam::{vec2, Vec3};

pub const MAX_TILT: f32 = 0.55; // rad; stick = 1 means this much tilt
pub const THRUST_RANGE: f32 = 0.7; // throttle = 1 adds 70% of hover thrust
pub const MAX_YAW_RATE: f32 = 2.0; // rad/s at full yaw stick
pub const DRAG: f32 = 0.35; // linear drag coefficient (1/s), shared with the simulator
pub const G: f32 = 9.81;

const KP: Vec3 = Vec3::new(2.2, 3.0, 2.2); // velocity P gains (x, y, z)
const KI: Vec3 = Vec3::new(0.8, 1.0, 0.8); // velocity I gains
const I_LIMIT: f32 = 6.0; // integral clamp, prevents wind-up
const KP_YAW: f32 = 3.0;
const SETPOINT_ACCEL: f32 = 6.0; // max rate of change of the velocity setpoint (m/s²)

/// Stick inputs, each in [-1, 1].
#[derive(Default, Clone, Copy, Debug)]
pub struct Inputs {
    pub throttle: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
}

/// What the autopilot knows about the drone (an "estimator" output in a real stack).
#[derive(Clone, Copy, Debug)]
pub struct State {
    pub pos: Vec3,
    pub vel: Vec3,
    pub yaw: f32,
}

#[derive(Clone, Copy, Debug)]
pub enum YawCmd {
    Rate(f32),
    Heading(f32),
}

#[derive(Clone, Copy, Debug)]
pub struct Setpoint {
    pub vel: Vec3, // world frame, m/s
    pub yaw: YawCmd,
}

#[derive(Default)]
pub struct FlightController {
    integral: Vec3,
    smooth_vel: Vec3, // rate-limited velocity setpoint
}

impl FlightController {
    pub fn compute(&mut self, dt: f32, s: &State, sp: &Setpoint) -> Inputs {
        let max_h = G * MAX_TILT.tan(); // max horizontal acceleration
        let max_z = THRUST_RANGE * G; // max vertical acceleration

        // Setpoint smoothing (like PX4's trajectory generator): the commanded velocity may only
        // change as fast as the drone can comfortably accelerate, so sudden decisions don't jerk it.
        let step = (sp.vel - self.smooth_vel).clamp_length_max(SETPOINT_ACCEL * dt);
        self.smooth_vel += step;
        let target = self.smooth_vel;

        // Desired acceleration: PI on velocity error + feedforward to cancel drag at the target speed
        let err = target - s.vel;
        let accel = err * KP + self.integral * KI + target * DRAG;

        // Anti-windup: only integrate an axis while its output is not saturated
        let h = vec2(accel.x, accel.z);
        if h.length() < max_h {
            self.integral.x += err.x * dt;
            self.integral.z += err.z * dt;
        }
        if accel.y.abs() < max_z {
            self.integral.y += err.y * dt;
        }
        self.integral = self.integral.clamp(Vec3::splat(-I_LIMIT), Vec3::splat(I_LIMIT));

        // World-frame horizontal acceleration -> body-frame tilt (a = g * tan(tilt))
        let h = h.clamp_length_max(max_h);
        let fwd = vec2(-s.yaw.sin(), -s.yaw.cos());
        let right = vec2(s.yaw.cos(), -s.yaw.sin());
        let tilt = |a: f32| (a / G).atan() / MAX_TILT;

        let yaw_rate = match sp.yaw {
            YawCmd::Rate(r) => r,
            YawCmd::Heading(target) => wrap_angle(target - s.yaw) * KP_YAW,
        };

        Inputs {
            throttle: accel.y.clamp(-max_z, max_z) / max_z,
            pitch: tilt(h.dot(fwd)).clamp(-1.0, 1.0),
            roll: tilt(h.dot(right)).clamp(-1.0, 1.0),
            yaw: (yaw_rate / MAX_YAW_RATE).clamp(-1.0, 1.0),
        }
    }

    /// Disturbance (m/s²) the integral term is currently cancelling, i.e. the estimated wind.
    pub fn wind_estimate(&self) -> Vec3 {
        -(self.integral * KI)
    }
}

pub fn wrap_angle(a: f32) -> f32 {
    (a + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Point-mass model of the drone: tilt -> horizontal accel, drag, constant wind.
    #[test]
    fn holds_velocity_against_wind() {
        let mut fc = FlightController::default();
        let mut s = State { pos: Vec3::ZERO, vel: Vec3::ZERO, yaw: 0.0 };
        let wind = Vec3::new(2.0, 0.0, -1.0);
        let sp = Setpoint { vel: Vec3::new(0.0, 0.0, -5.0), yaw: YawCmd::Rate(0.0) };
        let dt = 1.0 / 60.0;
        for _ in 0..(20.0 / dt) as usize {
            let i = fc.compute(dt, &s, &sp);
            let (pitch, roll) = (-i.pitch * MAX_TILT, -i.roll * MAX_TILT);
            let a = Vec3::new(-G * roll.tan(), i.throttle * THRUST_RANGE * G, G * pitch.tan());
            s.vel += (a + wind - s.vel * DRAG) * dt;
        }
        assert!((s.vel - sp.vel).length() < 0.2, "velocity {:?} did not converge", s.vel);
        assert!((fc.wind_estimate() - wind).length() < 0.5, "wind estimate {:?}", fc.wind_estimate());
    }
}
