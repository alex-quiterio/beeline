# Beeline

*An autonomous drone flying through a rainy noir city. It plans like a flight stack and steers like a honeybee.*

Beeline is a small drone flight simulator written in Rust. A quadcopter flies a loop of waypoints through a procedurally generated city full of moving hazards (sliding gates, rising lifts, spinning fans, patrol drones) and gusting wind. It does this with a layered autonomy stack modelled on real drone software (PX4 / ArduPilot), plus a navigation trick borrowed from bees.

```
cargo run --release          # fly it (press T for autopilot)
cargo test --release         # unit tests + a headless end-to-end flight
cargo test --release -- --ignored --nocapture   # benchmark across 10 cities
```

## Controls

| Key | Action |
|---|---|
| Arrow keys | Move forward / back / sideways (velocity command; release to hold position) |
| W / S | Climb / descend |
| A / D | Turn |
| T | Toggle autopilot |
| R | Reset to start |

In autopilot you see the planned route (red line on the ground), the lidar ring (red where something is close), the bee's **optic flow** bars at the bottom, and a wind compass comparing the **true wind** (white) with what the flight controller has **estimated** (red).

## Architecture

The crate is split so that the autonomy never touches the physics engine or the renderer directly, the way real flight software is separated from hardware:

```
                ┌─────────────────────────── nav (autonomy) ───────────────────────────┐
  waypoints ──► │ Mission ─► A* planner ─► Optic flow ─► Avoidance ─► Altitude        │ ──► velocity setpoint
                └──────────────────────────────▲───────────────────────────────────────┘          │
                                               │ Sensors trait (raycasts)                         ▼
┌──────────────────── sim (world) ─────────────┴───┐                    ┌──────── flight (controller) ────────┐
│ Rapier physics · airframe model · city · wind    │ ◄── stick inputs ─ │ rate limiter → PI + drag feedforward │
└──────────────────────────────────────────────────┘                    │ → tilt / throttle                    │
                     │                                                  └──────────────────────────────────────┘
                     ▼
            render (binary only): noir viewer
```

| Path | Role |
|---|---|
| `src/sim/` | The world: fixed-step Rapier physics, the airframe model, procedural city (`city.rs`), wind, and the `Sensors` implementation. No rendering, so it runs headless. |
| `src/nav/` | Autonomy. `planner.rs` (A* + string pulling), `optic_flow.rs` (bee behaviours), `avoidance.rs` (lidar collision prevention), `mod.rs` (the `Autopilot` that chains them). |
| `src/flight.rs` | Flight controller: velocity setpoint → sticks. Shared by the autopilot and manual mode. |
| `src/main.rs`, `src/render.rs` | The viewer: input, fixed-timestep loop with interpolated rendering, noir graphics. |
| `tests/headless_flight.rs` | Flies the full stack with no window; asserts a lap is completed with few collisions. |

Key design choices:

- **Sensors are a trait.** The autopilot only sees the world through `Sensors::raycast`. The same code runs in the viewer, in headless tests, and could run against another simulator or real hardware.
- **One flight controller for both modes.** Manual flying is "position mode": keys command a velocity and the same controller does the flying, so letting go of the keys makes the drone brake and hold its position in the wind.
- **Fixed timestep and interpolation.** Physics and control run at exactly 60 Hz for deterministic, testable behaviour. Rendering interpolates between steps, so it stays smooth on 120 Hz displays.
- **Deterministic worlds.** A seed fully defines a city, so any failure can be replayed exactly.

## How it flies

### Flight controller (`flight.rs`)
A cascade, as in PX4:
1. A **velocity setpoint** comes in and is **rate-limited**, so sudden decisions can't jerk the airframe.
2. A **PI controller** with **drag feedforward** turns the velocity error into a desired acceleration.
3. The integral term absorbs steady disturbances, which makes it a free **wind estimator** (the red arrow in the compass).
4. Acceleration is converted to **tilt** (`a = g·tan θ`) and **throttle**, with anti-windup when saturated.

### Autonomy layers (`nav/`)
1. **Mission:** cycle through the waypoints.
2. **Global planning:** A* on an occupancy grid of the known buildings, inflated by a safety margin. String pulling reduces the path to a few straight segments. The drone follows a "carrot" point, skips ahead when the next corner is visible, and replans every 2 s.
3. **Turn-aware speed:** it slows down in proportion to how sharply the route turns, so it takes corners tightly.
4. **Bee optic flow:** see below.
5. **Avoidance:** a 3-ring, 72-ray "lidar". Pass 1 dodges moving obstacles using **time to collision** and the obstacle's velocity *at the point the ray hit* (so spinning fan tips count). Pass 2 is a **PX4-style collision-prevention safety filter**: it removes any velocity pointing at something close, so the drone slides along walls instead of hitting them.
6. **Altitude:** cruise at 5 m, keep clear of whatever is below, back off from anything overhead, and climb over obstacles if boxed in for more than 1.5 s.

### The bees 🐝
Honeybees don't measure distances. They measure **optic flow**, how fast the world slides across each eye (for a side wall, roughly `speed / distance`). Mandyam Srinivasan's corridor experiments showed they follow two rules:

- **Centring:** balance the flow in the left and right eyes, which keeps them in the middle of the corridor.
- **Speed regulation:** hold the total flow at a set point, which slows them down automatically where space is tight.

Beeline's drone does exactly that with two side-looking sensors (`nav/optic_flow.rs`). With no explicit rule about street widths, it slows down in narrow streets and stays centred between buildings. The same idea flies real micro-drones (Franceschini's optic-flow regulator, EPFL's flying robots).

## Current results

`cargo test --release -- --ignored --nocapture` flies one lap in 10 different cities (seeds 1–10):

| | |
|---|---|
| Laps completed | **10 / 10** |
| Mean lap time | **76.9 s** |
| Collisions | **8 total** (6 patrol drones, 1 fan, 1 lift; **0 buildings**) |

The remaining collisions are mostly with patrol drones that move at up to ~18 m/s, faster than a purely reactive system can get out of the way of. That's the first item on the roadmap.

## Simplifications (where the model is honest about cheating)

- **Attitude is kinematic.** The airframe eases toward the commanded tilt; there are no motor torques, rotor inertia or rate loop.
- **Perfect state.** The autopilot gets exact position and velocity, with no IMU/GPS noise or estimator.
- **Range sensors stand in for vision**, including the bee's "eyes". Obstacle velocity is read from the physics engine rather than estimated by tracking.
- **The map is known in advance.** The planner gets the static building footprints at take-off.
- **Wind** is a smooth global field (a slowly rotating breeze plus sine gusts), not turbulence.

## Roadmap: ideas to improve the modelling

**Flight physics**
- [ ] Real quadrotor dynamics: four motor thrusts → torques → rigid-body rotation, a motor mixer, and an inner **attitude/rate PID loop** (the part PX4's `mc_rate_control` does).
- [ ] Quadratic drag, **ground effect**, rotor downwash on nearby surfaces, and **Dryden turbulence** in place of sine gusts.
- [ ] Battery model (voltage sag, capacity) and energy-aware route planning.

**Sensing and estimation**
- [ ] Sensor noise, latency and dropouts; an **EKF** fusing IMU, barometer and GPS (GPS degraded between tall buildings, an "urban canyon").
- [ ] **Real optic flow from rendered camera frames** (Lucas–Kanade on a low-resolution off-screen render) to replace the range-ray "eyes".
- [ ] Obstacle **tracking** (Kalman filter per object) to estimate hazard velocities instead of reading them from physics.

**Navigation**
- [ ] **Predictive avoidance** using velocity obstacles (ORCA) or a short-horizon **MPC**, to beat the fast patrol drones.
- [ ] **Mapping without a prior map**: build the occupancy grid from lidar while flying, and replan incrementally with **D\* Lite**.
- [ ] 3D planning, so going *over* a low building can be cheaper than going around it.
- [ ] More bee behaviour: **landing at constant optic flow** (bees slow down smoothly as they descend), **path integration** (odometry from optic flow) and **snapshot homing** to a remembered view.
- [ ] Swarms: multiple drones sharing the sky with flocking rules, and bee-colony-style task allocation.

**Tooling**
- [ ] Scenario files (city layout, hazards, wind, mission) and a CI job that tracks the benchmark numbers to catch regressions.
- [ ] Flight recording and replay, plus plots of setpoint vs actual.
- [ ] Train a learned policy (RL) headlessly through the same `Sensors` trait, running thousands of cities in parallel.
- [ ] A MAVLink/SITL bridge so a real PX4 autopilot can fly in this city.

## References

- M. V. Srinivasan et al., *Range perception through apparent image speed in freely flying honeybees* (1991), and *Honeybee navigation en route to the goal* (1996).
- N. Franceschini, F. Ruffier, J. Serres, *A bio-inspired flying robot sheds light on insect piloting abilities* (2007).
- PX4 Autopilot: multicopter position/velocity control and Collision Prevention ([docs.px4.io](https://docs.px4.io)).
- J. van den Berg et al., *Reciprocal n-body collision avoidance* (ORCA, 2011).

## Built with

[Rapier](https://rapier.rs) (physics, Apache-2.0) · [macroquad](https://macroquad.rs) (rendering, MIT/Apache-2.0) · [glam](https://github.com/bitshifter/glam-rs) (maths)
