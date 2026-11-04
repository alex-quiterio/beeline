//! Autonomy stack. Each layer runs every control step and refines a velocity setpoint:
//!
//!   1. Mission      - which waypoint are we going to?
//!   2. Planner      - A* route around the known (static) buildings, followed with a "carrot"
//!   3. Optic flow   - bee-style speed regulation and street centring
//!   4. Avoidance    - collision prevention + time-to-collision dodging of moving obstacles
//!   5. Altitude     - cruise height, terrain clearance, climb-over when boxed in
//!
//! The stack only sees the world through the [`Sensors`] trait, so it doesn't know or care whether
//! it is running in the simulator, in a headless test, or (one day) on real hardware.

pub mod avoidance;
pub mod optic_flow;
pub mod planner;

use crate::flight::{Setpoint, State, YawCmd};
use avoidance::SensorRay;
use glam::{vec2, vec3, Vec2, Vec3};
use optic_flow::Flow;
use planner::Grid;

pub const CRUISE_ALT: f32 = 5.0; // preferred flying height (m)
const CRUISE_SPEED: f32 = 7.0; // m/s
const CORNER_SPEED: f32 = 2.5; // speed when passing a path corner (m/s)
const BRAKE: f32 = 2.5; // comfortable deceleration used for speed planning (m/s²)
const REACHED: f32 = 2.0; // waypoint acceptance radius (m)
const REPLAN_EVERY: f32 = 2.0; // s
const CLEARANCE: f32 = 2.5; // minimum height above whatever is below (m)

/// What a range sensor found: distance, and the velocity of the thing it hit
/// (a real drone would estimate this by tracking the obstacle over time).
#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub dist: f32,
    pub vel: Vec3,
}

/// The autopilot's only window onto the world.
pub trait Sensors {
    fn raycast(&self, origin: Vec3, dir: Vec3, max_dist: f32) -> Option<Hit>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Status {
    Following,
    Avoiding,
    Climbing,
}

impl Status {
    pub fn label(self) -> &'static str {
        match self {
            Status::Following => "FOLLOWING ROUTE",
            Status::Avoiding => "AVOIDING",
            Status::Climbing => "CLIMBING OVER",
        }
    }
}

pub struct Autopilot {
    pub waypoints: Vec<Vec3>,
    pub current: usize,
    pub laps: usize,
    pub status: Status,
    pub path: Vec<Vec2>,
    pub rays: Vec<SensorRay>,
    pub flow: Flow,
    carrot: usize,
    grid: Grid,
    replan_timer: f32,
    stuck_time: f32,
    climb_extra: f32,
}

impl Autopilot {
    pub fn new(waypoints: Vec<Vec3>, grid: Grid) -> Self {
        Self {
            waypoints, current: 0, laps: 0, status: Status::Following, path: vec![], rays: vec![],
            flow: Flow::default(), carrot: 0, grid, replan_timer: 0.0, stuck_time: 0.0, climb_extra: 0.0,
        }
    }

    pub fn replan_now(&mut self) {
        self.replan_timer = 0.0;
    }

    pub fn update(&mut self, dt: f32, s: &State, sensors: &impl Sensors) -> Setpoint {
        let p2 = vec2(s.pos.x, s.pos.z);

        // --- 1. Mission
        let wp = self.waypoints[self.current];
        if p2.distance(vec2(wp.x, wp.z)) < REACHED {
            self.current = (self.current + 1) % self.waypoints.len();
            if self.current == 0 {
                self.laps += 1;
            }
            self.replan_now();
        }

        // --- 2. Global route (replanned periodically, so detours don't leave a stale route)
        let (route_dir, mut speed) = self.follow_route(dt, p2);
        // Turn-aware speed: the more the route bends away from where we're going, the slower we go,
        // so corners are taken tightly instead of swinging wide into the wall
        let ground_vel = vec3(s.vel.x, 0.0, s.vel.z);
        if ground_vel.length() > 1.5 {
            let alignment = ground_vel.normalize().dot(route_dir).max(0.0);
            speed = speed.min(CORNER_SPEED + (CRUISE_SPEED - CORNER_SPEED) * alignment.powi(3));
        }

        // --- 3. Optic flow: regulate speed and centre in the street
        let (v, flow) = optic_flow::regulate(sensors, s.pos, route_dir, speed);
        self.flow = flow;

        // --- 4. Avoidance
        let (mut v, avoidance) = avoidance::avoid(sensors, s.pos, s.vel, v);
        self.rays = avoidance.rays;

        // --- 5. Altitude: cruise height, clear of whatever is below, climbing if boxed in
        let progress = s.vel.dot(route_dir);
        let blocked = avoidance.active && speed > 2.0 && progress < 0.7;
        self.stuck_time = if blocked { (self.stuck_time + dt).min(2.0) } else { (self.stuck_time - dt).max(0.0) };
        if self.stuck_time > 1.5 {
            self.climb_extra = (self.climb_extra + 3.0 * dt).min(20.0);
        } else if !avoidance.active {
            self.climb_extra = (self.climb_extra - 0.7 * dt).max(0.0);
        }
        let floor = sensors.raycast(s.pos, -Vec3::Y, 100.0).map_or(0.0, |h| s.pos.y - h.dist);
        let target_alt = (CRUISE_ALT + self.climb_extra).max(floor + CLEARANCE);
        v.y = ((target_alt - s.pos.y) * 1.2).clamp(-2.5, 3.0);
        if let Some(c) = sensors.raycast(s.pos, Vec3::Y, 4.0)
            && c.dist < 2.5
        {
            v.y = v.y.min(-1.5 + c.vel.y.min(0.0)); // something overhead (or coming down): descend
        }

        self.status = if self.climb_extra > 0.5 {
            Status::Climbing
        } else if avoidance.active {
            Status::Avoiding
        } else {
            Status::Following
        };

        // Face the direction of travel; hold heading when nearly stopped
        let hv = vec2(v.x, v.z).clamp_length_max(CRUISE_SPEED + 3.0);
        let yaw = if hv.length() > 1.0 { YawCmd::Heading((-hv.x).atan2(-hv.y)) } else { YawCmd::Rate(0.0) };
        Setpoint { vel: vec3(hv.x, v.y, hv.y), yaw }
    }

    /// Plan (or reuse) the A* route and return the direction to the carrot and the planned speed.
    fn follow_route(&mut self, dt: f32, p2: Vec2) -> (Vec3, f32) {
        self.replan_timer -= dt;
        let off_route = self.path.get(self.carrot).is_none_or(|c| c.distance(p2) > 12.0);
        if self.replan_timer <= 0.0 || off_route {
            let goal = self.waypoints[self.current];
            let goal = vec2(goal.x, goal.z);
            self.path = self.grid.find_path(p2, goal).unwrap_or_else(|| vec![p2, goal]);
            self.carrot = 1;
            self.replan_timer = REPLAN_EVERY;
        }
        // Advance the carrot once reached, or as soon as the next corner is directly visible
        while self.carrot + 1 < self.path.len()
            && (p2.distance(self.path[self.carrot]) < 1.5 || self.grid.line_of_sight(p2, self.path[self.carrot + 1]))
        {
            self.carrot += 1;
        }
        // Speed profile: cruise, but slow down to take the next corner at CORNER_SPEED
        let to = self.path[self.carrot] - p2;
        let d = to.length();
        let speed = CRUISE_SPEED.min(CORNER_SPEED + (2.0 * BRAKE * d).sqrt());
        (vec3(to.x, 0.0, to.y) / d.max(0.01), speed)
    }
}
