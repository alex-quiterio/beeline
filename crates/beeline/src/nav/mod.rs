//! Autonomy stack.
//!
//! ```text
//!  Sensors ──► Perception ──► Mission ──► Guidance ──► velocity Setpoint ──► flight controller
//!              mapping        (where)     (how)
//!              beacon track
//! ```
//!
//! * **Perception** (every step, even in manual flight): lidar scans build the drone's own map
//!   (`mapping`), and the camera tracks the beacon's position and velocity.
//! * **Mission** (`mission`): a pluggable state machine that turns operator commands and what the
//!   drone knows into an [`mission::Objective`]: where to go, how high, whether to stop or land.
//! * **Guidance** refines that into a safe velocity, layer by layer:
//!   1. Planner    - A* on the learned map, followed with a "carrot" point (`planner`)
//!   2. Speed      - braking for corners and for stopping points
//!   3. Optic flow - bee-style speed regulation and street centring (`optic_flow`)
//!   4. Avoidance  - collision prevention + time-to-collision dodging (`avoidance`)
//!   5. Altitude   - cruise height, terrain clearance, climb-over, mission altitude, landing
//!
//! The stack perceives the world only through [`Sensors`], so it runs the same in the simulator,
//! in headless tests, or (one day) on real hardware.

pub mod avoidance;
pub mod explore;
pub mod mapping;
pub mod mission;
pub mod optic_flow;
pub mod planner;
pub mod search;

pub use mission::{Command, Mission, Phase, RadioMsg, Retrieve};

use crate::flight::{Setpoint, YawCmd};
use crate::sensors::{Payload, Sensors};
use crate::vehicle::State;
use avoidance::SensorRay;
use glam::{Vec2, Vec3, vec2, vec3};
use mission::{Ctx, Objective, Radio};
use optic_flow::Flow;
use planner::Grid;

pub const CRUISE_ALT: f32 = 5.0; // preferred flying height (m)
const CRUISE_SPEED: f32 = 7.0; // m/s
const CORNER_SPEED: f32 = 2.5; // speed when passing a path corner (m/s)
const BRAKE: f32 = 2.5; // comfortable deceleration used for speed planning (m/s²)
const REPLAN_EVERY: f32 = 1.0; // s
const CLEARANCE: f32 = 2.5; // minimum height above whatever is below (m)
const SCAN_EVERY: f32 = 0.1; // lidar mapping rate (s)
const RADIO_LINES: usize = 6;

/// The beacon as the drone believes it to be.
#[derive(Clone, Copy, Debug)]
pub struct Track {
    pub pos: Vec3,
    pub vel: Vec3,
    pub seen_at: f32,
}

pub struct Autopilot {
    /// The map the drone has built so far.
    pub map: Grid,
    pub home: Vec3,
    /// The current objective's goal (for display and logging).
    pub goal: Option<Vec2>,
    pub track: Option<Track>,
    pub path: Vec<Vec2>,
    pub rays: Vec<SensorRay>,
    pub flow: Flow,
    pub avoiding: bool,
    pub time: f32,
    /// When the operator first gave an order.
    pub mission_start: Option<f32>,
    /// The latest lidar sweep (for the flight recorder).
    pub last_scan: Option<mapping::Scan>,
    mission: Box<dyn Mission>,
    radio: Radio,
    payload: Payload,
    planned_goal: Vec2,
    carrot: usize,
    replan_timer: f32,
    scan_timer: f32,
    stuck_time: f32,
    climb_extra: f32,
}

impl Autopilot {
    /// An autopilot flying the default mission (retrieve the beacon).
    pub fn new(home: Vec3, half_extent: f32) -> Self {
        Self::with_mission(home, half_extent, Box::new(Retrieve::default()))
    }

    pub fn with_mission(home: Vec3, half_extent: f32, mission: Box<dyn Mission>) -> Self {
        let mut ap = Self {
            map: Grid::new(half_extent, 1.0),
            home,
            goal: None,
            track: None,
            path: vec![],
            rays: vec![],
            flow: Flow::default(),
            avoiding: false,
            time: 0.0,
            mission_start: None,
            last_scan: None,
            mission,
            radio: Radio::default(),
            payload: Payload::None,
            planned_goal: Vec2::ZERO,
            carrot: 0,
            replan_timer: 0.0,
            scan_timer: 0.0,
            stuck_time: 0.0,
            climb_extra: 0.0,
        };
        ap.radio.say("BEELINE ONLINE. NO MAP. AWAITING ORDERS.");
        ap
    }

    /// Start from a map other drones built (see `fleet.rs`) instead of a blank one.
    pub fn load_prior(&mut self, map: Grid, flights: usize) {
        assert_eq!(map.size(), self.map.size(), "prior map must cover the same area");
        self.map = map;
        let pct = self.map.explored() * 100.0;
        self.radio.say(&format!("FLEET INTEL: {flights} EARLIER FLIGHT(S), {pct:.0}% OF THE CITY PRE-MAPPED."));
    }

    pub fn phase(&self) -> Phase {
        self.mission.phase()
    }

    /// Every radio message so far, oldest first.
    pub fn radio(&self) -> &[RadioMsg] {
        &self.radio.messages
    }

    /// The last few radio messages, for display.
    pub fn recent_radio(&self) -> &[RadioMsg] {
        let m = &self.radio.messages;
        &m[m.len().saturating_sub(RADIO_LINES)..]
    }

    pub fn sees_beacon(&self) -> bool {
        self.track.is_some_and(|t| self.time - t.seen_at < 0.3)
    }

    /// An operator instruction, passed to the mission (which acknowledges it on the radio).
    pub fn command(&mut self, cmd: Command) {
        self.mission_start.get_or_insert(self.time);
        let pos = self.last_scan.as_ref().map_or(self.home, |s| s.pos); // latest known position
        let mut ctx = Ctx {
            time: self.time,
            pos,
            home: self.home,
            map: &self.map,
            payload: self.payload,
            track: self.track,
            radio: &mut self.radio,
        };
        self.mission.command(cmd, &mut ctx);
        self.replan_timer = 0.0;
    }

    /// Sense the world and let the mission react. Runs even when a human is flying.
    pub fn perceive(&mut self, dt: f32, s: &State, sensors: &impl Sensors) {
        self.time += dt;
        self.radio.now = self.time;
        self.scan_timer -= dt;
        if self.scan_timer <= 0.0 {
            self.last_scan = Some(mapping::integrate_scan(&mut self.map, sensors, s.pos, self.time));
            self.scan_timer = SCAN_EVERY;
        }
        if let Some(b) = sensors.spot_beacon(s.pos) {
            self.observe_beacon(b);
        }
        self.payload = sensors.payload();
        let mut ctx = Ctx {
            time: self.time,
            pos: s.pos,
            home: self.home,
            map: &self.map,
            payload: self.payload,
            track: self.track,
            radio: &mut self.radio,
        };
        self.mission.observe(dt, &mut ctx);
    }

    /// Perceive, ask the mission for an objective, and turn it into a safe velocity setpoint.
    pub fn update(&mut self, dt: f32, s: &State, sensors: &impl Sensors) -> Setpoint {
        self.perceive(dt, s, sensors);
        let mut ctx = Ctx {
            time: self.time,
            pos: s.pos,
            home: self.home,
            map: &self.map,
            payload: self.payload,
            track: self.track,
            radio: &mut self.radio,
        };
        let objective = self.mission.objective(dt, &mut ctx);
        self.guide(dt, s, sensors, objective)
    }

    /// Guidance: route, speed, optic flow, avoidance and altitude for one objective.
    fn guide(&mut self, dt: f32, s: &State, sensors: &impl Sensors, obj: Objective) -> Setpoint {
        self.goal = obj.goal;
        let Some(goal) = obj.goal else {
            self.path.clear();
            self.avoiding = false;
            return Setpoint { vel: Vec3::ZERO, yaw: YawCmd::Rate(0.0) }; // hover in place
        };
        let p2 = vec2(s.pos.x, s.pos.z);

        // --- Route and speed
        let (route_dir, carrot_dist, final_leg) = self.follow_route(dt, p2, goal);
        let mut speed = CRUISE_SPEED.min(CORNER_SPEED + (2.0 * BRAKE * carrot_dist).sqrt());
        if final_leg && obj.stop {
            speed = speed.min((2.0 * BRAKE * carrot_dist).sqrt()); // come to a stop at the goal
        }
        // Turn-aware: slow down in proportion to how sharply the route bends
        let ground_vel = vec3(s.vel.x, 0.0, s.vel.z);
        if ground_vel.length() > 1.5 {
            let alignment = ground_vel.normalize().dot(route_dir).max(0.0);
            speed = speed.min(CORNER_SPEED + (CRUISE_SPEED - CORNER_SPEED) * alignment.powi(3));
        }

        // --- Bee optic flow, then avoidance as the final safety filter
        let (v, flow) = optic_flow::regulate(sensors, s.pos, route_dir, speed);
        self.flow = flow;
        let (mut v, avoidance) = avoidance::avoid(sensors, s.pos, s.vel, v);
        self.rays = avoidance.rays;
        self.avoiding = avoidance.active;

        // --- Altitude
        let progress = s.vel.dot(route_dir);
        let blocked = avoidance.active && speed > 2.0 && progress < 0.7;
        self.stuck_time = if blocked { (self.stuck_time + dt).min(2.0) } else { (self.stuck_time - dt).max(0.0) };
        if self.stuck_time > 1.5 {
            self.climb_extra = (self.climb_extra + 3.0 * dt).min(20.0); // boxed in: go over
        } else if !avoidance.active {
            self.climb_extra = (self.climb_extra - 0.7 * dt).max(0.0);
        }
        // Highest thing under any part of the airframe (centre and four corners), so a corner
        // can't clip a roof edge the centre ray misses
        let floor = [(0.0, 0.0), (0.8, 0.8), (0.8, -0.8), (-0.8, 0.8), (-0.8, -0.8)]
            .iter()
            .filter_map(|&(dx, dz)| {
                let from = s.pos + vec3(dx, 0.0, dz);
                sensors.raycast(from, -Vec3::Y, 100.0).map(|h| from.y - h.dist)
            })
            .fold(0.0, f32::max);
        v.y = if obj.land {
            if carrot_dist < 1.5 { -1.0 } else { 0.0 }
        } else {
            let target = match obj.altitude {
                Some(a) => a.max(floor + 1.0),
                None => (CRUISE_ALT + self.climb_extra).max(floor + CLEARANCE),
            };
            ((target - s.pos.y) * 1.2).clamp(-2.5, 3.0)
        };
        if let Some(c) = sensors.raycast(s.pos, Vec3::Y, 4.0)
            && c.dist < 2.5
        {
            v.y = v.y.min(-1.5 + c.vel.y.min(0.0)); // something overhead (or coming down): descend
        }

        // Face the direction of travel; hold heading when nearly stopped
        let hv = vec2(v.x, v.z).clamp_length_max(CRUISE_SPEED + 3.0);
        let yaw = if hv.length() > 1.0 { YawCmd::Heading((-hv.x).atan2(-hv.y)) } else { YawCmd::Rate(0.0) };
        Setpoint { vel: vec3(hv.x, v.y, hv.y), yaw }
    }

    /// Follow the route to `goal`: straight if the map shows a clear line, otherwise A*.
    /// Returns the direction to the carrot, the distance to it, and whether it's the final point.
    fn follow_route(&mut self, dt: f32, p2: Vec2, goal: Vec2) -> (Vec3, f32, bool) {
        self.replan_timer -= dt;
        if self.map.line_of_sight(p2, goal) {
            self.path = vec![p2, goal];
            self.carrot = 1;
        } else if self.replan_timer <= 0.0
            || goal.distance(self.planned_goal) > 4.0
            || self.path.get(self.carrot).is_none_or(|c| c.distance(p2) > 12.0)
        {
            self.path = self.map.find_path(p2, goal).unwrap_or_else(|| vec![p2, goal]);
            self.carrot = 1;
            self.planned_goal = goal;
            self.replan_timer = REPLAN_EVERY;
        }
        // Advance the carrot once reached, or as soon as the next corner is directly visible
        while self.carrot + 1 < self.path.len()
            && (p2.distance(self.path[self.carrot]) < 1.5 || self.map.line_of_sight(p2, self.path[self.carrot + 1]))
        {
            self.carrot += 1;
        }
        let to = self.path[self.carrot] - p2;
        let d = to.length();
        (vec3(to.x, 0.0, to.y) / d.max(0.01), d, self.carrot + 1 == self.path.len())
    }

    fn observe_beacon(&mut self, b: Vec3) {
        match &mut self.track {
            Some(t) if self.time - t.seen_at < 1.0 => {
                let dt = self.time - t.seen_at;
                if dt > 1e-4 {
                    t.vel = t.vel.lerp((b - t.pos) / dt, 0.2); // smoothed velocity estimate
                }
                t.pos = b;
                t.seen_at = self.time;
            }
            _ => self.track = Some(Track { pos: b, vel: Vec3::ZERO, seen_at: self.time }),
        }
    }
}

/// Format seconds as m:ss.
pub fn clock(t: f32) -> String {
    format!("{}:{:02}", (t / 60.0) as u32, (t % 60.0) as u32)
}
