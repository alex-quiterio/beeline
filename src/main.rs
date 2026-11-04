//! Beeline viewer: map menu, then a fixed-rate simulation drawn at the display's rate.
//!
//! ```text
//! beeline                                 # menu
//! beeline --map docks --seed 7 --retrieve # straight into a mission
//! beeline --map downtown --retrieve --skip 60 --screenshot shot.png --after 2
//! ```

mod render;

use beeline::fleet;
use beeline::flight::{FlightController, Setpoint, YawCmd};
use beeline::nav::{Autopilot, Command, Phase};
use beeline::sim::city::MapKind;
use beeline::sim::{DT, World};
use beeline::telemetry::FlightRecorder;
use beeline::vehicle::MAX_YAW_RATE;
use macroquad::prelude::*;
use render::{Hud, Renderer};
use std::path::{Path, PathBuf};

const DEFAULT_SEED: u64 = 7;
const LOG_DIR: &str = "logs";
const MANUAL_SPEED: f32 = 8.0; // m/s at full stick
const MANUAL_CLIMB: f32 = 3.0; // m/s

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

/// One deployment of the drone into a city.
struct Session {
    world: World,
    autopilot: Autopilot,
    controller: FlightController,
    recorder: FlightRecorder,
    auto: bool,
    acc: f32,
    fleet_flights: usize,
    log_path: Option<PathBuf>,
    toast: Option<(String, f64)>,
}

impl Session {
    fn new(map: MapKind, seed: u64) -> Self {
        let world = World::new(map, seed);
        let mut autopilot = Autopilot::new(world.home, world.city.half_extent);
        let mut fleet_flights = 0;
        if let Some((prior, n)) = fleet::load_prior(Path::new(LOG_DIR), map.name(), seed, world.city.half_extent) {
            autopilot.load_prior(prior, n);
            fleet_flights = n;
        }
        let recorder = FlightRecorder::new(&world);
        Self {
            world,
            autopilot,
            controller: FlightController::default(),
            recorder,
            auto: false,
            acc: 0.0,
            fleet_flights,
            log_path: None,
            toast: None,
        }
    }

    fn command(&mut self, cmd: Command) {
        self.autopilot.command(cmd);
        self.auto = true;
    }

    /// One fixed simulation step: sense -> plan -> control -> physics -> record.
    fn step(&mut self) {
        let state = self.world.drone_state();
        let setpoint = if self.auto {
            self.autopilot.update(DT, &state, &self.world)
        } else {
            self.autopilot.perceive(DT, &state, &self.world); // keep mapping while a human flies
            manual_setpoint(state.yaw)
        };
        let input = self.controller.compute(DT, &state, &setpoint);
        self.world.step(input);
        self.recorder.record(&self.world, &self.autopilot, &setpoint, self.auto, self.controller.wind_estimate());
        if self.autopilot.phase() == Phase::Done && self.log_path.is_none() {
            self.save_log(); // a finished mission files its own report
        }
    }

    fn advance(&mut self, frame_dt: f32) {
        self.acc += frame_dt.min(0.1);
        while self.acc >= DT {
            self.step();
            self.acc -= DT;
        }
    }

    fn save_log(&mut self) {
        match self.recorder.save(Path::new(LOG_DIR)) {
            Ok(path) => {
                self.toast = Some((format!("FLIGHT LOG SAVED  {}", path.display()), get_time() + 5.0));
                self.log_path = Some(path);
            }
            Err(e) => self.toast = Some((format!("COULD NOT SAVE LOG: {e}"), get_time() + 5.0)),
        }
    }

    fn hud(&self) -> Hud<'_> {
        Hud {
            auto: self.auto,
            fleet_flights: self.fleet_flights,
            toast: self.toast.as_ref().filter(|(_, until)| get_time() < *until).map(|(t, _)| t.as_str()),
            log_path: self.log_path.as_deref(),
        }
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

/// Command-line options (all optional).
#[derive(Default)]
struct Options {
    map: Option<MapKind>,
    seed: Option<u64>,
    retrieve: bool,
    skip: f32,
    screenshot: Option<String>,
    after: f32,
}

fn options() -> Options {
    let mut o = Options { after: 2.0, ..Default::default() };
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        let next = args.get(i + 1).cloned().unwrap_or_default();
        match args[i].as_str() {
            "--map" => o.map = MapKind::parse(&next),
            "--seed" => o.seed = next.parse().ok(),
            "--skip" => o.skip = next.parse().unwrap_or(0.0),
            "--screenshot" => o.screenshot = Some(next),
            "--after" => o.after = next.parse().unwrap_or(2.0),
            "--retrieve" => {
                o.retrieve = true;
                i += 1;
                continue;
            }
            other => {
                eprintln!("unknown option {other}");
                i += 1;
                continue;
            }
        }
        i += 2;
    }
    o
}

enum Screen {
    Menu { selected: usize, seed: u64 },
    Flying(Box<Session>),
}

#[macroquad::main(window_conf)]
async fn main() {
    let opts = options();
    let mut renderer = Renderer::new();
    let mut screen = match opts.map {
        Some(map) => {
            let mut s = Session::new(map, opts.seed.unwrap_or(DEFAULT_SEED));
            if opts.retrieve {
                s.command(Command::Retrieve);
            }
            for _ in 0..(opts.skip / DT) as usize {
                s.step(); // fast-forward headless
            }
            Screen::Flying(Box::new(s))
        }
        None => Screen::Menu { selected: 0, seed: opts.seed.unwrap_or(DEFAULT_SEED) },
    };
    let start = get_time();
    let demo = opts.screenshot.is_some(); // unattended capture: ignore the keyboard
    let pressed = |k: KeyCode| !demo && is_key_pressed(k);

    loop {
        let dt = get_frame_time();
        let mut next: Option<Screen> = None;
        match &mut screen {
            Screen::Menu { selected, seed } => {
                if pressed(KeyCode::Left) || pressed(KeyCode::Up) {
                    *selected = 0;
                }
                if pressed(KeyCode::Right) || pressed(KeyCode::Down) {
                    *selected = 1;
                }
                if pressed(KeyCode::N) {
                    *seed = macroquad::rand::gen_range(1, 100_000);
                }
                let pick = if pressed(KeyCode::Key1) {
                    Some(0)
                } else if pressed(KeyCode::Key2) {
                    Some(1)
                } else if pressed(KeyCode::Enter) {
                    Some(*selected)
                } else {
                    None
                };
                let logs: Vec<usize> = MapKind::ALL
                    .iter()
                    .map(|m| fleet::matching_logs(Path::new(LOG_DIR), m.name(), *seed).len())
                    .collect();
                renderer.draw_menu(*selected, *seed, &logs);
                if let Some(i) = pick {
                    renderer.reset();
                    next = Some(Screen::Flying(Box::new(Session::new(MapKind::ALL[i], *seed))));
                }
            }
            Screen::Flying(s) => {
                if pressed(KeyCode::F) {
                    s.command(Command::Retrieve);
                }
                if pressed(KeyCode::H) {
                    s.command(Command::Hold);
                }
                if pressed(KeyCode::B) {
                    s.command(Command::ReturnHome);
                }
                if pressed(KeyCode::T) {
                    s.auto = !s.auto;
                }
                if pressed(KeyCode::L) {
                    s.save_log();
                }
                s.advance(dt);
                let alpha = s.acc / DT;
                renderer.draw(&s.world, &s.autopilot, &s.controller, alpha, dt, s.hud());
                if pressed(KeyCode::R) {
                    renderer.reset();
                    next = Some(Screen::Flying(Box::new(Session::new(s.world.map, s.world.seed))));
                }
                if pressed(KeyCode::M) {
                    next = Some(Screen::Menu {
                        selected: MapKind::ALL.iter().position(|m| *m == s.world.map).unwrap_or(0),
                        seed: s.world.seed,
                    });
                }
            }
        }

        if let Some(path) = &opts.screenshot
            && get_time() - start > opts.after as f64
        {
            let mut shot = get_screen_data();
            for px in shot.get_image_data_mut() {
                px[3] = 255; // the framebuffer's alpha is not meant to be shown
            }
            shot.export_png(path);
            println!("screenshot saved to {path}");
            break;
        }
        if let Some(n) = next {
            screen = n;
        }
        next_frame().await;
    }
}
