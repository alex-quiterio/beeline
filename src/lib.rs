//! # Beeline
//!
//! An autonomous drone flight simulator. The crate is split into three layers that only talk
//! through small interfaces, the way real drone software is built:
//!
//! * [`sim`]    - the world: Rapier physics, the airframe model, a procedural city, wind, sensors.
//! * [`nav`]    - autonomy: A* planning, bee-inspired optic flow, collision avoidance. Sees the
//!   world only through the [`nav::Sensors`] trait.
//! * [`flight`] - the flight controller: velocity setpoint in, stick inputs out.
//! * [`telemetry`] - the flight recorder, writing standard PX4 ULog files.
//! * [`fleet`]  - fleet intel: rebuild the terrain from earlier flights' logs before take-off.
//!
//! None of these depend on a renderer; the viewer lives in the binary (`src/main.rs`, `src/render.rs`).

pub mod fleet;
pub mod flight;
pub mod nav;
pub mod sensors;
pub mod sim;
pub mod telemetry;
pub mod vehicle;
