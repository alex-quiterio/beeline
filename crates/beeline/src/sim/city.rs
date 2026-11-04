//! Procedural maps: a street grid filled per map style, plus moving hazards and gusting wind.
//! Deterministic for a given seed, so test runs and bug reports are reproducible.

use glam::{Vec2, Vec3, vec2, vec3};
use rapier3d::na::{Isometry3, Vector3};
use rapier3d::prelude::*;

const SPAWN_CLEAR: f32 = 14.0; // keep hazards away from the home pad

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MapKind {
    Downtown,
    Docks,
}

impl MapKind {
    pub const ALL: [MapKind; 2] = [MapKind::Downtown, MapKind::Docks];

    pub fn name(self) -> &'static str {
        match self {
            MapKind::Downtown => "DOWNTOWN",
            MapKind::Docks => "THE DOCKS",
        }
    }

    pub fn tagline(self) -> &'static str {
        match self {
            MapKind::Downtown => "Rain-slick canyons of stone and glass. Tight streets, tall towers.",
            MapKind::Docks => "Container mazes, warehouses and gantry cranes by the black water.",
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.to_ascii_lowercase().as_str() {
            "downtown" | "1" => Some(MapKind::Downtown),
            "docks" | "the-docks" | "2" => Some(MapKind::Docks),
            _ => None,
        }
    }

    /// Street spacing (m) and blocks from the centre to the edge.
    fn layout(self) -> (f32, i32) {
        match self {
            MapKind::Downtown => (16.0, 9),
            MapKind::Docks => (24.0, 7),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Shape {
    Box(Vec3), // full size
    Ball(f32), // radius
}

/// What an obstacle is, so the renderer can style it (the simulation doesn't care).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Building { shade: f32, windows: bool },
    Container { shade: f32 },
    Crane,
    Gate,
    Lift,
    Fan,
    Patrol,
}

#[derive(Clone, Copy, Debug)]
enum Motion {
    Static,
    /// base + axis * amp * sin(t * speed + phase)
    Slide {
        base: Vec3,
        axis: Vec3,
        amp: f32,
        speed: f32,
        phase: f32,
    },
    /// Rotates in place around `axis`
    Spin {
        base: Vec3,
        axis: Vec3,
        speed: f32,
        phase: f32,
    },
}

pub struct Obstacle {
    pub body: RigidBodyHandle,
    pub shape: Shape,
    pub kind: Kind,
    motion: Motion,
}

pub struct City {
    pub kind: MapKind,
    pub obstacles: Vec<Obstacle>,
    /// Static ground footprints `(centre, half_size)`: ground truth, NOT given to the autopilot.
    pub footprints: Vec<(Vec2, Vec2)>,
    /// Street spacing (m); streets run along every multiple of this.
    pub block: f32,
    /// Blocks from the centre to the edge.
    pub grid: i32,
    pub half_extent: f32,
}

impl City {
    pub fn generate(kind: MapKind, seed: u64, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) -> Self {
        let (block, grid) = kind.layout();
        let mut city =
            Self { kind, obstacles: vec![], footprints: vec![], block, grid, half_extent: grid as f32 * block + 6.0 };
        let mut rng = Rng(seed);
        match kind {
            MapKind::Downtown => city.downtown(&mut rng, bodies, colliders),
            MapKind::Docks => city.docks(&mut rng, bodies, colliders),
        }
        city.hazards(&mut rng, bodies, colliders);
        city
    }

    /// Centres of every block, as (x, z).
    fn blocks(&self) -> impl Iterator<Item = Vec2> + use<> {
        let (g, b) = (self.grid, self.block);
        (-g..g).flat_map(move |gx| (-g..g).map(move |gz| vec2((gx as f32 + 0.5) * b, (gz as f32 + 0.5) * b)))
    }

    fn downtown(&mut self, rng: &mut Rng, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        let reach = self.grid as f32 * self.block * 1.2;
        for c in self.blocks().collect::<Vec<_>>() {
            if rng.range(0.0, 1.0) < 0.12 {
                continue;
            } // a park
            let centrality = (1.0 - c.length() / reach).max(0.0);
            let (w, d) = (rng.range(6.0, 10.0), rng.range(6.0, 10.0));
            let h = rng.range(4.0, 10.0 + 30.0 * centrality);
            let kind = Kind::Building { shade: rng.range(0.35, 0.65), windows: true };
            self.add_static(bodies, colliders, vec3(w, h, d), vec3(c.x, h / 2.0, c.y), kind, true);
        }
    }

    fn docks(&mut self, rng: &mut Rng, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        for c in self.blocks().collect::<Vec<_>>() {
            let r = rng.range(0.0, 1.0);
            if r < 0.3 {
                // Warehouse: big, low, dark
                let (w, d, h) = (rng.range(12.0, 18.0), rng.range(8.0, 16.0), rng.range(6.0, 11.0));
                let kind = Kind::Building { shade: rng.range(0.3, 0.5), windows: false };
                self.add_static(bodies, colliders, vec3(w, h, d), vec3(c.x, h / 2.0, c.y), kind, true);
            } else if r < 0.7 {
                // Container yard: three rows of stacks, with narrow alleys between them
                for row in 0..3 {
                    let z = c.y - 6.0 + row as f32 * 6.0;
                    let mut x = c.x - 9.0;
                    while x + 5.0 <= c.x + 9.0 {
                        if rng.range(0.0, 1.0) < 0.8 {
                            let h = 2.6 * (1 + rng.below(3)) as f32;
                            let kind = Kind::Container { shade: rng.range(0.2, 0.6) };
                            self.add_static(
                                bodies,
                                colliders,
                                vec3(5.0, h, 2.5),
                                vec3(x + 2.5, h / 2.0, z),
                                kind,
                                true,
                            );
                        }
                        x += 5.4;
                    }
                }
            } else if r < 0.82 {
                // Gantry crane: four legs and a beam high overhead (you can fly under it)
                let h = rng.range(14.0, 20.0);
                for (lx, lz) in [(-6.0, -3.0), (-6.0, 3.0), (6.0, -3.0), (6.0, 3.0)] {
                    self.add_static(
                        bodies,
                        colliders,
                        vec3(1.0, h, 1.0),
                        vec3(c.x + lx, h / 2.0, c.y + lz),
                        Kind::Crane,
                        true,
                    );
                }
                for lz in [-3.0, 3.0] {
                    self.add_static(
                        bodies,
                        colliders,
                        vec3(14.0, 1.2, 1.2),
                        vec3(c.x, h, c.y + lz),
                        Kind::Crane,
                        false,
                    );
                }
            } // else: an empty lot
        }
    }

    /// Moving hazards on the streets, scaled to the map size.
    fn hazards(&mut self, rng: &mut Rng, bodies: &mut RigidBodySet, colliders: &mut ColliderSet) {
        let (b, g) = (self.block, self.grid);
        let n = 2 * g as usize;
        let street = |k: i32| k as f32 * b;

        // Sliding gates: walls that slide across a street, opening and closing it
        for _ in 0..n {
            let (z, x) = (street(rng.street(g)), street(rng.street(g)) + b / 2.0);
            if x.hypot(z) < SPAWN_CLEAR {
                continue;
            }
            let base = vec3(x, 4.5, z);
            let motion = Motion::Slide {
                base,
                axis: Vec3::Z,
                amp: b * 0.3,
                speed: rng.range(0.4, 0.9),
                phase: rng.range(0.0, 6.0),
            };
            self.add(bodies, colliders, Shape::Box(vec3(0.6, 9.0, b * 0.45)), Kind::Gate, base, motion);
        }
        // Lifts: platforms rising and falling at junctions
        for _ in 0..n {
            let (x, z) = (street(rng.street(g)), street(rng.street(g)));
            if x.hypot(z) < SPAWN_CLEAR {
                continue;
            }
            let base = vec3(x, 8.0, z);
            let motion =
                Motion::Slide { base, axis: Vec3::Y, amp: 7.0, speed: rng.range(0.3, 0.7), phase: rng.range(0.0, 6.0) };
            self.add(bodies, colliders, Shape::Box(vec3(5.0, 0.5, 5.0)), Kind::Lift, base, motion);
        }
        // Fans: two crossed blades spinning across a street, sweeping through cruise altitude
        for _ in 0..n * 2 / 3 {
            let (x, z) = (street(rng.street(g)) + b / 2.0, street(rng.street(g)));
            if x.hypot(z) < SPAWN_CLEAR {
                continue;
            }
            let (base, speed) = (vec3(x, 6.0, z), rng.range(0.8, 2.0));
            for phase in [0.0, std::f32::consts::FRAC_PI_2] {
                let motion = Motion::Spin { base, axis: Vec3::X, speed, phase };
                self.add(bodies, colliders, Shape::Box(vec3(0.4, 0.8, b * 0.45)), Kind::Fan, base, motion);
            }
        }
        // Patrol drones: balls flying up and down streets at roughly cruise altitude
        for i in 0..n {
            let lane = street(rng.street(g));
            let height = rng.range(3.5, 7.0);
            let (base, axis) =
                if i % 2 == 0 { (vec3(0.0, height, lane), Vec3::X) } else { (vec3(lane, height, 0.0), Vec3::Z) };
            let amp = rng.range(0.3, 0.8) * g as f32 * b;
            let motion = Motion::Slide { base, axis, amp, speed: rng.range(0.1, 0.25), phase: rng.range(0.0, 6.0) };
            self.add(bodies, colliders, Shape::Ball(0.7), Kind::Patrol, base, motion);
        }
    }

    fn add_static(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        size: Vec3,
        center: Vec3,
        kind: Kind,
        footprint: bool,
    ) {
        self.add(bodies, colliders, Shape::Box(size), kind, center, Motion::Static);
        if footprint {
            self.footprints.push((vec2(center.x, center.z), vec2(size.x, size.z) / 2.0));
        }
    }

    fn add(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        shape: Shape,
        kind: Kind,
        pos: Vec3,
        motion: Motion,
    ) {
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
                Motion::Slide { base, axis, amp, speed, phase } => {
                    (base + axis * amp * (t * speed + phase).sin(), Vec3::ZERO)
                }
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

/// Small deterministic PRNG (SplitMix64), so worlds don't depend on any engine's RNG.
#[derive(Clone)]
pub(crate) struct Rng(pub(crate) u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    pub(crate) fn range(&mut self, a: f32, b: f32) -> f32 {
        a + (self.next() >> 40) as f32 / (1u64 << 24) as f32 * (b - a)
    }

    pub(crate) fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }

    /// A street index, not on the city edge.
    pub(crate) fn street(&mut self, grid: i32) -> i32 {
        -grid + 1 + self.below(2 * grid as u64 - 1) as i32
    }
}
