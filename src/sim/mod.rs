//! The simulated world: Rapier physics, the drone's airframe model, the city, and the sensors.
//! Runs at a fixed time step and has no rendering, so it works headless in tests.

pub mod city;

use crate::flight::{Inputs, State, DRAG, G, MAX_TILT, MAX_YAW_RATE, THRUST_RANGE};
use crate::nav::{Hit, Sensors};
use city::City;
use glam::{Quat, Vec3};
use rapier3d::na::{UnitQuaternion, Vector3};
use rapier3d::prelude::*;

pub const DT: f32 = 1.0 / 60.0;
const SPAWN: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const TILT_RESPONSE: f32 = 6.0; // 1/s, how fast the airframe reaches the commanded tilt

#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub pos: Vec3,
    pub rot: Quat,
}

impl Pose {
    pub fn lerp(self, other: Pose, t: f32) -> Pose {
        Pose { pos: self.pos.lerp(other.pos, t), rot: self.rot.slerp(other.rot, t) }
    }

    fn of(body: &RigidBody) -> Pose {
        let (t, r) = (body.translation(), body.rotation());
        Pose { pos: Vec3::new(t.x, t.y, t.z), rot: Quat::from_xyzw(r.i, r.j, r.k, r.w) }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Collision {
    pub time: f32,
    pub kind: Option<city::Kind>,
}

pub struct World {
    pub city: City,
    pub time: f32,
    /// Every time the drone hit something (ground excluded): when, and what.
    pub collisions: Vec<Collision>,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    pipeline: PhysicsPipeline,
    params: IntegrationParameters,
    islands: IslandManager,
    broad: DefaultBroadPhase,
    narrow: NarrowPhase,
    joints: ImpulseJointSet,
    multibody: MultibodyJointSet,
    ccd: CCDSolver,
    query: QueryPipeline,
    ground: ColliderHandle,
    drone: RigidBodyHandle,
    drone_collider: ColliderHandle,
    attitude: (f32, f32, f32), // yaw, pitch, roll
    touching: bool,
    // Poses before the latest step, for smooth rendering between physics steps
    prev_drone: Pose,
    prev_obstacles: Vec<Pose>,
}

impl World {
    pub fn new(seed: u64) -> Self {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let ground = colliders.insert(ColliderBuilder::cuboid(300.0, 0.1, 300.0).translation(vector![0.0, -0.1, 0.0]));
        let city = City::generate(seed, &mut bodies, &mut colliders);

        // The drone: attitude is driven by the airframe model below, so physics rotation is locked
        let drone = bodies.insert(
            RigidBodyBuilder::dynamic().translation(vector![SPAWN.x, SPAWN.y, SPAWN.z]).linear_damping(DRAG).lock_rotations(),
        );
        let drone_collider = colliders.insert_with_parent(ColliderBuilder::cuboid(0.6, 0.1, 0.6), drone, &mut bodies);

        let mut world = Self {
            city, time: 0.0, collisions: vec![], bodies, colliders,
            pipeline: PhysicsPipeline::new(),
            params: IntegrationParameters { dt: DT, ..Default::default() },
            islands: IslandManager::new(),
            broad: DefaultBroadPhase::new(),
            narrow: NarrowPhase::new(),
            joints: ImpulseJointSet::new(),
            multibody: MultibodyJointSet::new(),
            ccd: CCDSolver::new(),
            query: QueryPipeline::new(),
            ground, drone, drone_collider,
            attitude: (0.0, 0.0, 0.0),
            touching: false,
            prev_drone: Pose { pos: SPAWN, rot: Quat::IDENTITY },
            prev_obstacles: vec![],
        };
        world.query.update(&world.colliders);
        world.snapshot();
        world
    }

    pub fn reset_drone(&mut self) {
        let b = &mut self.bodies[self.drone];
        b.set_translation(vector![SPAWN.x, SPAWN.y, SPAWN.z], true);
        b.set_linvel(vector![0.0, 0.0, 0.0], true);
        self.attitude = (0.0, 0.0, 0.0);
    }

    pub fn wind(&self) -> Vec3 {
        city::wind(self.time)
    }

    pub fn drone_state(&self) -> State {
        let b = &self.bodies[self.drone];
        let (t, v) = (b.translation(), b.linvel());
        State { pos: Vec3::new(t.x, t.y, t.z), vel: Vec3::new(v.x, v.y, v.z), yaw: self.attitude.0 }
    }

    /// Advance the world by one fixed step with the given stick inputs.
    pub fn step(&mut self, input: Inputs) {
        self.snapshot();
        self.time += DT;
        self.city.update(self.time, &mut self.bodies);

        // Airframe model ("angle mode"): sticks set a target tilt, the drone eases toward it
        let (yaw, pitch, roll) = &mut self.attitude;
        *yaw += input.yaw * MAX_YAW_RATE * DT;
        *pitch += (-input.pitch * MAX_TILT - *pitch) * TILT_RESPONSE * DT;
        *roll += (-input.roll * MAX_TILT - *roll) * TILT_RESPONSE * DT;
        let q = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), *yaw)
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), *pitch)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), *roll);

        // Thrust along the body's up axis, compensated for tilt so tilting doesn't lose altitude
        let wind = city::wind(self.time);
        let b = &mut self.bodies[self.drone];
        b.set_rotation(q, true);
        let hover = b.mass() * G / (pitch.cos() * roll.cos());
        let wind = wind * b.mass();
        b.reset_forces(true);
        b.add_force(q * vector![0.0, hover * (1.0 + THRUST_RANGE * input.throttle), 0.0], true);
        b.add_force(vector![wind.x, wind.y, wind.z], true);

        self.pipeline.step(
            &vector![0.0, -G, 0.0], &self.params, &mut self.islands, &mut self.broad, &mut self.narrow,
            &mut self.bodies, &mut self.colliders, &mut self.joints, &mut self.multibody, &mut self.ccd,
            Some(&mut self.query), &(), &(),
        );

        // Count collisions (new contacts with anything but the ground)
        let (ground, me) = (self.ground, self.drone_collider);
        let other = self.narrow.contact_pairs_with(me)
            .filter(|p| p.has_any_active_contact && p.collider1 != ground && p.collider2 != ground)
            .map(|p| if p.collider1 == me { p.collider2 } else { p.collider1 })
            .next();
        if let (Some(c), false) = (other, self.touching) {
            let body = self.colliders[c].parent();
            let kind = self.city.obstacles.iter().find(|o| Some(o.body) == body).map(|o| o.kind);
            self.collisions.push(Collision { time: self.time, kind });
        }
        self.touching = other.is_some();
    }

    /// Drone pose, interpolated `alpha` (0..1) of the way from the previous step to the current one.
    pub fn drone_pose(&self, alpha: f32) -> Pose {
        self.prev_drone.lerp(Pose::of(&self.bodies[self.drone]), alpha)
    }

    pub fn obstacle_pose(&self, i: usize, alpha: f32) -> Pose {
        let now = Pose::of(&self.bodies[self.city.obstacles[i].body]);
        self.prev_obstacles.get(i).map_or(now, |p| p.lerp(now, alpha))
    }

    pub fn yaw(&self) -> f32 {
        self.attitude.0
    }

    fn snapshot(&mut self) {
        self.prev_drone = Pose::of(&self.bodies[self.drone]);
        self.prev_obstacles = self.city.obstacles.iter().map(|o| Pose::of(&self.bodies[o.body])).collect();
    }
}

impl Sensors for World {
    fn raycast(&self, origin: Vec3, dir: Vec3, max_dist: f32) -> Option<Hit> {
        let ray = Ray::new(point![origin.x, origin.y, origin.z], vector![dir.x, dir.y, dir.z]);
        let filter = QueryFilter::default().exclude_rigid_body(self.drone);
        self.query
            .cast_ray(&self.bodies, &self.colliders, &ray, max_dist, true, filter)
            .map(|(handle, dist)| {
                // Velocity of the exact point that was hit (so a spinning blade's tip counts as moving)
                let point = ray.point_at(dist);
                let v = self.colliders[handle].parent().map_or(Vector::zeros(), |p| self.bodies[p].velocity_at_point(&point));
                Hit { dist, vel: Vec3::new(v.x, v.y, v.z) }
            })
    }
}
