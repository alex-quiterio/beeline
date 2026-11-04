//! End-to-end checks, no window: the full stack (sim + autopilot + flight controller) flies the city.

use beeline::flight::FlightController;
use beeline::nav::{planner::Grid, Autopilot};
use beeline::sim::{city::HALF_EXTENT, Collision, World, DT};

struct Run {
    lap_time: Option<f32>,
    collisions: Vec<Collision>,
}

/// Fly one lap of the city generated from `seed`, giving up after `max_time` seconds.
fn fly_lap(seed: u64, max_time: f32) -> Run {
    let mut world = World::new(seed);
    let grid = Grid::from_footprints(HALF_EXTENT, 1.0, &world.city.footprints, 1.8);
    let mut autopilot = Autopilot::new(world.city.waypoints.clone(), grid);
    let mut controller = FlightController::default();

    while world.time < max_time && autopilot.laps == 0 {
        let state = world.drone_state();
        assert!(state.pos.is_finite(), "seed {seed}: simulation blew up at t={:.1}", world.time);
        let setpoint = autopilot.update(DT, &state, &world);
        let input = controller.compute(DT, &state, &setpoint);
        world.step(input);
    }
    Run { lap_time: (autopilot.laps > 0).then_some(world.time), collisions: world.collisions }
}

#[test]
fn autopilot_completes_a_lap_of_the_city() {
    let run = fly_lap(7, 240.0);
    println!("lap time {:?} s, collisions {:?}", run.lap_time, run.collisions);
    assert!(run.lap_time.is_some(), "did not finish a lap");
    assert!(run.collisions.len() <= 3, "too many collisions: {:?}", run.collisions);
}

/// Benchmark across many cities: `cargo test --release -- --ignored --nocapture`
#[test]
#[ignore]
fn benchmark_many_cities() {
    let (mut finished, mut hits, mut total_time) = (0, 0, 0.0);
    for seed in 1..=10 {
        let run = fly_lap(seed, 300.0);
        let kinds: Vec<String> = run.collisions.iter()
            .map(|c| format!("{:?}", c.kind).split([' ', '(']).nth(1).unwrap_or("?").to_string())
            .collect();
        println!("seed {seed:>2}: lap {:>7}  collisions {:>2}  {kinds:?}",
            run.lap_time.map_or("DNF".into(), |t| format!("{t:.1}s")), run.collisions.len());
        finished += run.lap_time.is_some() as u32;
        total_time += run.lap_time.unwrap_or(0.0);
        hits += run.collisions.len();
    }
    println!("finished {finished}/10, mean lap {:.1}s, {hits} collisions total", total_time / finished.max(1) as f32);
}
