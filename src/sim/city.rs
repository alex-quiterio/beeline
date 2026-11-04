//! Procedural city: buildings on a street grid plus moving hazards and gusting wind.
//! Deterministic for a given seed, so test runs and bug reports are reproducible.

use glam::{vec2, vec3, Vec2, Vec3};
use rapier3d::na::{Isometry3, Vector3};
use rapier3d::prelude::*;

pub const BLOCK: f32 = 16.0; // grid spacing: streets run along every multiple of BLOCK
const GRID: i32 = 5; // city spans -GRID..GRID blocks in each direction
pub const HALF_EXTENT: f32 = GRID as f32 * BLOCK + 20.0;
const WAYPOINT_ALT: f32 = 5.0;
const SPAWN_CLEAR: f32 = 12.0; // keep moving hazards away from the spawn point

#[derive(Clone, Copy, Debug)]
pub enum Shape {
    Box(Vec3), // full size
    Ball(f32), // radius
}

/// What an obstacle is, so the renderer can style it (the simulation doesn't care).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Building { shade: f32 },
    Gate,
    Lift,
    Fan,
    Patrol,
}

#[derive(Clone, Copy, Debug)]
enum Motion {
    Static,
    /// base + axis * amp * sin(t * speed + phase)
    Slide { base: Vec3, axis: Vec3, amp: f32, speed: f32, phase: f32 },
    /// Rotates in place around `axis`
    Spin { base: Vec3, axis: Vec3, speed: f32, phase: f32 },
}

pub struct Obstacle {
    pub body: RigidBodyHandle,
    pub shape: Shape,
    pub kind: Kind,
    motion: Motion,
}

pub struct City {
    pub obstacles: Vec<Obstacle>,
    pub waypoints: Vec<Vec3>,
    /// Static building footprints `(centre, half_size)` on the ground plane: the "map" a drone
    /// would have before take-off. Moving hazards are deliberately not in it.
    pub footprints: Vec<(Vec2, Vec2)>,
}

impl City {
    pub fn generate(seed: u64, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> Self {
        let mut city = Self { obstacles: vec![], waypoints: vec![], footprints: vec![] };
        let mut rng = Rng(seed);
        let street = |k: i32| k as f32 * BLOCK;

        // Buildings: one per block, taller toward the centre, ~15% left empty as parks
        for gx in -GRID..GRID {
            for gz in -GRID..GRID {
                if rng.range(0.0, 1.0) < 0.15 { continue; }
                let (cx, cz) = ((gx as f32 + 0.5) * BLOCK, (gz as f32 + 0.5) * BLOCK);
                let centrality = (1.0 - cx.hypot(cz) / 110.0).max(0.0);
                let (w, d) = (rng.range(6.0, 10.0), rng.range(6.0, 10.0));
                let h = rng.range(4.0, 10.0 + 22.0 * centrality);
                let kind = Kind::Building { shade: rng.range(0.35, 0.65) };
                city.add(bodies, colliders, Shape::Box(vec3(w, h, d)), kind, vec3(cx, h / 2.0, cz), Motion::Static);
                city.footprints.push((vec2(cx, cz), vec2(w, d) / 2.0));
            }
        }

        // Sliding gates: walls that slide across a street, opening and closing it
        for _ in 0..8 {
            let (z, x) = (street(rng.street()), street(rng.street()) + BLOCK / 2.0);
            if x.hypot(z) < SPAWN_CLEAR { continue; }
            let base = vec3(x, 4.5, z);
            let motion = Motion::Slide { base, axis: Vec3::Z, amp: 5.0, speed: rng.range(0.4, 0.9), phase: rng.range(0.0, 6.0) };
            city.add(bodies, colliders, Shape::Box(vec3(0.6, 9.0, 7.0)), Kind::Gate, base, motion);
        }

        // Lifts: platforms rising and falling at junctions
        for _ in 0..8 {
            let (x, z) = (street(rng.street()), street(rng.street()));
            if x.hypot(z) < SPAWN_CLEAR { continue; }
            let base = vec3(x, 8.0, z);
            let motion = Motion::Slide { base, axis: Vec3::Y, amp: 7.0, speed: rng.range(0.3, 0.7), phase: rng.range(0.0, 6.0) };
            city.add(bodies, colliders, Shape::Box(vec3(5.0, 0.5, 5.0)), Kind::Lift, base, motion);
        }

        // Fans: two crossed blades spinning across a street, sweeping through cruise altitude
        for _ in 0..6 {
            let (x, z) = (street(rng.street()) + BLOCK / 2.0, street(rng.street()));
            if x.hypot(z) < SPAWN_CLEAR { continue; }
            let (base, speed) = (vec3(x, 6.0, z), rng.range(0.8, 2.0));
            for phase in [0.0, std::f32::consts::FRAC_PI_2] {
                let motion = Motion::Spin { base, axis: Vec3::X, speed, phase };
                city.add(bodies, colliders, Shape::Box(vec3(0.4, 0.8, 7.0)), Kind::Fan, base, motion);
            }
        }

        // Patrol drones: balls flying up and down streets at roughly cruise altitude
        for i in 0..10 {
            let lane = street(rng.street());
            let height = rng.range(3.5, 7.0);
            let (base, axis) = if i % 2 == 0 { (vec3(0.0, height, lane), Vec3::X) } else { (vec3(lane, height, 0.0), Vec3::Z) };
            let motion = Motion::Slide { base, axis, amp: rng.range(25.0, 60.0), speed: rng.range(0.1, 0.3), phase: rng.range(0.0, 6.0) };
            city.add(bodies, colliders, Shape::Ball(0.7), Kind::Patrol, base, motion);
        }

        // Waypoints: a loop of street junctions around the centre
        for i in 0..6 {
            let a = i as f32 / 6.0 * std::f32::consts::TAU;
            let r = rng.range(30.0, 65.0);
            let snap = |v: f32| (v / BLOCK).round() * BLOCK;
            let wp = vec3(snap(a.cos() * r), WAYPOINT_ALT, snap(a.sin() * r));
            if !city.waypoints.contains(&wp) {
                city.waypoints.push(wp);
            }
        }
        city
    }

    fn add(&mut self, bodies: &mut RigidBodySet, colliders: &mut ColliderSet, shape: Shape, kind: Kind, pos: Vec3, motion: Motion) {
        let builder = match motion {
            Motion::Static => RigidBodyBuilder::fixed(),
            _ => RigidBodyBuilder::kinematic_position_based(),
        };
        let body = bodies.insert(builder.translation(vector![pos.x, pos.y, pos.z]));
        let collider = match shape {
            Shape::Box(s) => ColliderBuilder::cuboid(s.x / 2.0, s.y / 2.0, s.z / 2.0),
            Shape::Ball(r) => ColliderBuilder::ball(r),
        };
        colliders.insert_with_parent(collider, body, bodies);
        self.obstacles.push(Obstacle { body, shape, kind, motion });
    }

    /// Move the kinematic hazards to where they should be at time `t`.
    pub fn update(&self, t: f32, bodies: &mut RigidBodySet) {
        for o in &self.obstacles {
            let (pos, rot) = match o.motion {
                Motion::Static => continue,
                Motion::Slide { base, axis, amp, speed, phase } => (base + axis * amp * (t * speed + phase).sin(), Vec3::ZERO),
                Motion::Spin { base, axis, speed, phase } => (base, axis * (t * speed + phase)),
            };
            let iso = Isometry3::new(Vector3::new(pos.x, pos.y, pos.z), Vector3::new(rot.x, rot.y, rot.z));
            bodies[o.body].set_next_kinematic_position(iso);
        }
    }
}

/// Wind as an acceleration (m/s²) at time `t`: a slowly turning breeze with periodic gusts.
pub fn wind(t: f32) -> Vec3 {
    let dir = t * 0.05;
    let gust = ((t * 0.7).sin() * (t * 0.23).sin()).max(0.0);
    vec3(dir.cos(), 0.0, dir.sin()) * (1.0 + 3.0 * gust)
}

/// Small deterministic PRNG (SplitMix64), so the city doesn't depend on any engine's RNG.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn range(&mut self, a: f32, b: f32) -> f32 {
        a + (self.next() >> 40) as f32 / (1u64 << 24) as f32 * (b - a)
    }

    /// A street index, not on the city edge.
    fn street(&mut self) -> i32 {
        -GRID + 1 + (self.next() % (2 * GRID as u64 - 1)) as i32
    }
}
