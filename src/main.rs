//! Beeline viewer: runs the simulation at a fixed rate and draws it at the display's rate.

mod render;

use beeline::flight::{FlightController, Setpoint, YawCmd, MAX_YAW_RATE};
use beeline::nav::{planner::Grid, Autopilot};
use beeline::sim::{city::HALF_EXTENT, World, DT};
use macroquad::prelude::*;
use render::Renderer;

const SEED: u64 = 7;
const MANUAL_SPEED: f32 = 8.0; // m/s at full stick
const MANUAL_CLIMB: f32 = 3.0; // m/s
const PLAN_MARGIN: f32 = 1.8; // how far the planner keeps routes from building walls (m)

fn window_conf() -> Conf {
    Conf {
        window_title: "Beeline".into(),
        window_width: 1280,
        window_height: 800,
        high_dpi: true,
        sample_count: 4,
        ..Default::default()
    }
}

fn key(k: KeyCode) -> f32 {
    if is_key_down(k) { 1.0 } else { 0.0 }
}

/// Manual flight is "position mode": keys command a velocity, release them and the drone brakes
/// and holds its position (against the wind) using the same flight controller as the autopilot.
fn manual_setpoint(yaw: f32) -> Setpoint {
    let forward = vec3(-yaw.sin(), 0.0, -yaw.cos());
    let right = vec3(yaw.cos(), 0.0, -yaw.sin());
    let vel = forward * (key(KeyCode::Up) - key(KeyCode::Down)) * MANUAL_SPEED
        + right * (key(KeyCode::Right) - key(KeyCode::Left)) * MANUAL_SPEED
        + Vec3::Y * (key(KeyCode::W) - key(KeyCode::S)) * MANUAL_CLIMB;
    Setpoint { vel, yaw: YawCmd::Rate((key(KeyCode::A) - key(KeyCode::D)) * MAX_YAW_RATE) }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut world = World::new(SEED);
    let grid = Grid::from_footprints(HALF_EXTENT, 1.0, &world.city.footprints, PLAN_MARGIN);
    let mut autopilot = Autopilot::new(world.city.waypoints.clone(), grid);
    let mut controller = FlightController::default();
    let mut renderer = Renderer::new();
    let mut auto_mode = false;
    let mut acc = 0.0;

    loop {
        if is_key_pressed(KeyCode::R) {
            world.reset_drone();
            autopilot.replan_now();
        }
        if is_key_pressed(KeyCode::T) {
            auto_mode = !auto_mode;
            autopilot.replan_now();
        }

        // Fixed-step simulation: sense -> plan -> control -> physics
        acc += get_frame_time().min(0.1);
        while acc >= DT {
            let state = world.drone_state();
            let setpoint = if auto_mode { autopilot.update(DT, &state, &world) } else { manual_setpoint(state.yaw) };
            let input = controller.compute(DT, &state, &setpoint);
            world.step(input);
            acc -= DT;
        }

        // Render, interpolating between the last two physics steps for smooth motion
        renderer.draw(&world, &autopilot, &controller, auto_mode, acc / DT, get_frame_time());
        next_frame().await;
    }
}
