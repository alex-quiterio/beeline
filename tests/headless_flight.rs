//! End-to-end checks, no window: the full stack (sim + autopilot + flight controller + recorder).

use beeline::fleet::{TerrainPrior, Ulog};
use beeline::flight::FlightController;
use beeline::nav::planner::CellState;
use beeline::nav::{Autopilot, Command, Phase};
use beeline::sim::city::MapKind;
use beeline::sim::{Collision, DT, World};
use beeline::telemetry::FlightRecorder;

struct Run {
    delivered_at: Option<f32>,
    found_at: Option<f32>,
    collisions: Vec<Collision>,
    explored: f32,
    log: Vec<u8>,
}

/// Order a retrieval mission in an unknown city and fly until delivered or `max_time` runs out.
fn retrieve(map: MapKind, seed: u64, max_time: f32, prior: Option<&[u8]>) -> Run {
    let mut world = World::new(map, seed);
    let mut autopilot = Autopilot::new(world.home, world.city.half_extent);
    if let Some(bytes) = prior {
        let mut p = TerrainPrior::new(world.city.half_extent);
        p.absorb(&Ulog::parse(bytes).unwrap());
        autopilot.load_prior(p.to_map(), 1);
    }
    let mut controller = FlightController::default();
    let mut recorder = FlightRecorder::new(&world);
    autopilot.command(Command::Retrieve);
    let mut found_at = None;

    while world.time < max_time && world.delivered_at.is_none() {
        let state = world.drone_state();
        assert!(state.pos.is_finite(), "{map:?}/{seed}: simulation blew up at t={:.1}", world.time);
        let setpoint = autopilot.update(DT, &state, &world);
        let input = controller.compute(DT, &state, &setpoint);
        world.step(input);
        recorder.record(&world, &autopilot, &setpoint, true, controller.wind_estimate());
        if found_at.is_none() && autopilot.phase() == Phase::Chase {
            found_at = Some(world.time);
        }
    }
    Run {
        delivered_at: world.delivered_at,
        found_at,
        collisions: world.collisions,
        explored: autopilot.map.explored(),
        log: recorder.bytes().to_vec(),
    }
}

fn report(map: MapKind, seed: u64, r: &Run) {
    let t = |x: Option<f32>| x.map_or("    —".into(), |t| format!("{t:5.0}s"));
    println!(
        "{:<10} seed {seed:>2}: found {}  delivered {}  explored {:>3.0}%  collisions {}",
        map.name(),
        t(r.found_at),
        t(r.delivered_at),
        r.explored * 100.0,
        r.collisions.len()
    );
}

#[test]
fn retrieves_the_beacon_downtown() {
    let r = retrieve(MapKind::Downtown, 7, 900.0, None);
    report(MapKind::Downtown, 7, &r);
    assert!(r.delivered_at.is_some(), "beacon not delivered");
    assert!(r.collisions.len() <= 6, "too many collisions: {:?}", r.collisions);
}

#[test]
fn retrieves_the_beacon_at_the_docks() {
    let r = retrieve(MapKind::Docks, 7, 900.0, None);
    report(MapKind::Docks, 7, &r);
    assert!(r.delivered_at.is_some(), "beacon not delivered");
    assert!(r.collisions.len() <= 6, "too many collisions: {:?}", r.collisions);
}

/// The flight log is a valid ULog with the standard topics, and other drones can rebuild the
/// terrain from it: walls in the rebuilt map must sit on real buildings.
#[test]
fn flight_log_rebuilds_the_terrain() {
    let r = retrieve(MapKind::Downtown, 3, 120.0, None);
    let log = Ulog::parse(&r.log).expect("valid ULog");
    assert_eq!(log.info.get("sys_name").map(String::as_str), Some("Beeline"));

    let world = World::new(MapKind::Downtown, 3);
    let mut prior = TerrainPrior::new(world.city.half_extent);
    let scans = prior.absorb(&log);
    assert!(scans > 500, "obstacle_distance scans in the log: {scans}");
    let map = prior.to_map();
    assert!(map.explored() > 0.05, "prior covers {:.1}%", map.explored() * 100.0);

    // Every rebuilt wall should be on (or within a cell or two of) a real building
    let (mut walls, mut real) = (0, 0);
    for y in 0..map.size() {
        for x in 0..map.size() {
            if map.state((x, y)) != CellState::Wall {
                continue;
            }
            walls += 1;
            let p = map.center_of((x, y));
            if world.city.footprints.iter().any(|(c, h)| {
                let d = (p - *c).abs() - *h;
                d.x < 1.5 && d.y < 1.5
            }) {
                real += 1;
            }
        }
    }
    println!("rebuilt walls: {walls}, on real buildings: {real}");
    assert!(walls > 100 && real as f32 / walls as f32 > 0.9, "{real}/{walls} rebuilt walls are real");
}

/// Benchmark: `cargo test --release -- --ignored --nocapture`
/// Each city is flown twice: blind, then again with fleet intel from the first flight's log.
#[test]
#[ignore]
fn benchmark_retrieval() {
    for map in MapKind::ALL {
        let (mut blind, mut informed, mut hits) = (vec![], vec![], 0);
        for seed in 1..=5 {
            let first = retrieve(map, seed, 900.0, None);
            report(map, seed, &first);
            let second = retrieve(map, seed, 900.0, Some(&first.log));
            print!("  with fleet intel -> ");
            report(map, seed, &second);
            blind.extend(first.delivered_at);
            informed.extend(second.delivered_at);
            hits += first.collisions.len();
        }
        let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len().max(1) as f32;
        println!(
            "{}: delivered {}/5 blind (mean {:.0}s, {hits} collisions), {}/5 with fleet intel (mean {:.0}s)\n",
            map.name(),
            blind.len(),
            mean(&blind),
            informed.len(),
            mean(&informed)
        );
    }
}
