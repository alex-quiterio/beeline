//! Fly a retrieval mission headless and save the flight log.
//!
//! ```text
//! cargo run --release --example mission -- docks 7          # map, seed
//! cargo run --release --example mission -- downtown 3 600   # ...and a time limit (s)
//! ```
//! Logs from earlier runs of the same city (in `logs/`) are used as fleet intel.

use beeline::fleet;
use beeline::flight::FlightController;
use beeline::nav::{Autopilot, Command, clock};
use beeline::sim::city::MapKind;
use beeline::sim::{DT, World};
use beeline::telemetry::FlightRecorder;
use std::path::Path;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let map = args.first().and_then(|s| MapKind::parse(s)).unwrap_or(MapKind::Downtown);
    let seed = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(7);
    let limit: f32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(900.0);

    let mut world = World::new(map, seed);
    let mut autopilot = Autopilot::new(world.home, world.city.half_extent);
    if let Some((prior, n)) = fleet::load_prior(Path::new("logs"), map.name(), seed, world.city.half_extent) {
        autopilot.load_prior(prior, n);
    }
    let mut controller = FlightController::default();
    let mut recorder = FlightRecorder::new(&world);
    autopilot.command(Command::Retrieve);

    let mut printed = 0;
    let started = std::time::Instant::now();
    while world.time < limit && world.delivered_at.is_none() {
        let state = world.drone_state();
        let setpoint = autopilot.update(DT, &state, &world);
        let input = controller.compute(DT, &state, &setpoint);
        world.step(input);
        recorder.record(&world, &autopilot, &setpoint, true, controller.wind_estimate());
        for m in &autopilot.radio()[printed..] {
            println!("{:>6}  {}", clock(m.time), m.text);
        }
        printed = autopilot.radio().len();
        if (world.time / 60.0).floor() != ((world.time - DT) / 60.0).floor() {
            let p = state.pos;
            println!(
                "{:>6}  · {} · {:.0}% mapped · at ({:.0}, {:.0}, {:.0}) · {:.1}x real time",
                clock(world.time),
                autopilot.phase().label(),
                autopilot.map.explored() * 100.0,
                p.x,
                p.y,
                p.z,
                world.time / started.elapsed().as_secs_f32()
            );
        }
    }

    println!(
        "\n{} seed {seed}: {}, {:.0}% of the city mapped, {} collisions",
        map.name(),
        world
            .delivered_at
            .map_or(format!("not delivered after {}", clock(limit)), |t| format!("delivered at {}", clock(t))),
        autopilot.map.explored() * 100.0,
        world.collisions.len()
    );
    let mut kinds = std::collections::BTreeMap::new();
    for c in &world.collisions {
        let k = c
            .kind
            .map_or("unknown".to_string(), |k| format!("{k:?}").split([' ', '{']).next().unwrap_or("?").to_string());
        *kinds.entry(k).or_insert(0) += 1;
    }
    if !kinds.is_empty() {
        println!("collisions by kind: {kinds:?}");
    }
    match recorder.save(Path::new("logs")) {
        Ok(p) => println!("flight log: {}", p.display()),
        Err(e) => eprintln!("could not save the flight log: {e}"),
    }
}
