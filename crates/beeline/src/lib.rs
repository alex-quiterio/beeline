//! # Beeline
//!
//! The headless core of an autonomous drone simulator, layered the way real drone software is.
//! Dependencies point one way only (top depends on bottom):
//!
//! ```text
//!   telemetry · fleet             flight recorder (PX4 ULog) · terrain from other drones' logs
//!          │
//!   sim ─────────┐   nav          the world (physics, city, beacon) · autonomy (mission + guidance)
//!          │     │    │
//!          │     └► flight        flight controller: velocity setpoint -> sticks
//!          │          │
//!   sensors · vehicle             shared interfaces: what can be sensed, what the airframe is
//! ```
//!
//! * [`vehicle`]   - airframe constants, stick [`vehicle::Inputs`], estimated [`vehicle::State`].
//! * [`sensors`]   - the [`sensors::Sensors`] trait: the autopilot's only window onto the world.
//! * [`flight`]    - the flight controller.
//! * [`nav`]       - perception, pluggable [`nav::Mission`]s, and guidance.
//! * [`sim`]       - Rapier physics, the procedural cities, the beacon; implements `Sensors`.
//! * [`telemetry`] - the flight recorder, writing standard PX4 ULog files.
//! * [`fleet`]     - rebuild the terrain from earlier flights' logs before take-off.
//!
//! `nav` and `sim` never import each other. This crate has no graphics dependency at all: the
//! viewer is a separate crate (`crates/viewer`) built on top of it.

pub mod fleet;
pub mod flight;
pub mod nav;
pub mod sensors;
pub mod sim;
pub mod telemetry;
pub mod vehicle;
