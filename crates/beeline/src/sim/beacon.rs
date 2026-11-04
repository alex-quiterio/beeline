//! The purple beacon: the thing the drone must find and bring home.
//! It wanders the street grid, and when the drone gets close it runs.

use super::city::{City, Rng};
use glam::{Vec2, Vec3, vec2, vec3};

pub const GRAB_RADIUS: f32 = 1.6; // m, the drone's "magnet"
const WANDER_SPEED: f32 = 2.0; // m/s
const FLEE_SPEED: f32 = 4.2; // m/s, slower than the drone, so it can be caught
const FLEE_RADIUS: f32 = 15.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BeaconState {
    Free,
    Carried,
    Delivered,
}

type Junction = (i32, i32);

pub struct Beacon {
    pub pos: Vec3,
    pub state: BeaconState,
    from: Junction,
    to: Junction,
    progress: f32, // 0..1 along the street from `from` to `to`
    rng: Rng,
    block: f32,
    limit: i32, // junction indices run -limit..=limit
    t: f32,
}

impl Beacon {
    /// Start at a random junction in the outer part of the city, far from home.
    pub fn new(city: &City, seed: u64) -> Self {
        let mut rng = Rng(seed ^ 0xBEAC_0000);
        let limit = city.grid - 1;
        let from = loop {
            let j = (rng.street(city.grid), rng.street(city.grid));
            if ((j.0 * j.0 + j.1 * j.1) as f32).sqrt() >= limit as f32 * 0.6 {
                break j;
            }
        };
        let mut beacon = Self {
            pos: Vec3::ZERO,
            state: BeaconState::Free,
            from,
            to: from,
            progress: 0.0,
            rng,
            block: city.block,
            limit,
            t: 0.0,
        };
        beacon.to = beacon.pick_next(from, None, None);
        beacon.place();
        beacon
    }

    pub fn update(&mut self, dt: f32, drone: Vec3) {
        self.t += dt;
        if self.state == BeaconState::Carried {
            self.pos = drone - Vec3::Y * 0.7;
        }
        if self.state != BeaconState::Free {
            return;
        }

        let fleeing = drone.distance(self.pos) < FLEE_RADIUS;
        if fleeing
            && self.junction(self.to).distance(vec2(drone.x, drone.z))
                < self.junction(self.from).distance(vec2(drone.x, drone.z))
        {
            // Running toward the drone: turn around
            (self.from, self.to) = (self.to, self.from);
            self.progress = 1.0 - self.progress;
        }
        self.progress += if fleeing { FLEE_SPEED } else { WANDER_SPEED } * dt / self.block;
        while self.progress >= 1.0 {
            self.progress -= 1.0;
            let prev = self.from;
            self.from = self.to;
            self.to = self.pick_next(self.from, Some(prev), fleeing.then_some(drone));
        }
        self.place();
    }

    fn place(&mut self) {
        let p = self.junction(self.from).lerp(self.junction(self.to), self.progress);
        self.pos = vec3(p.x, 3.5 + (self.t * 1.3).sin() * 0.8, p.y);
    }

    fn junction(&self, j: Junction) -> Vec2 {
        vec2(j.0 as f32, j.1 as f32) * self.block
    }

    /// Next junction: never straight back unless it's a dead end; away from the drone if fleeing.
    fn pick_next(&mut self, at: Junction, prev: Option<Junction>, flee_from: Option<Vec3>) -> Junction {
        let mut options: Vec<Junction> = [(1, 0), (-1, 0), (0, 1), (0, -1)]
            .iter()
            .map(|d| (at.0 + d.0, at.1 + d.1))
            .filter(|j| j.0.abs() <= self.limit && j.1.abs() <= self.limit && Some(*j) != prev)
            .collect();
        if options.is_empty() {
            return prev.unwrap_or(at);
        }
        if let Some(d) = flee_from {
            let d = vec2(d.x, d.z);
            options.sort_by(|a, b| self.junction(*b).distance(d).total_cmp(&self.junction(*a).distance(d)));
            return options[0];
        }
        options[self.rng.below(options.len() as u64) as usize]
    }
}
