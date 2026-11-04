//! Missions decide **where** to go; the autopilot's guidance layers decide **how** to get there.
//!
//! A mission is a state machine that receives operator [`Command`]s and, every step, turns what
//! the drone knows ([`Ctx`]) into an [`Objective`]. To add a scenario (search-and-rescue,
//! delivery, inspection...), implement [`Mission`]; nothing in guidance or the simulator changes.

use super::planner::Grid;
use super::search::SearchMap;
use super::{Track, explore};
use crate::sensors::Payload;
use glam::{Vec2, Vec3, vec2};

/// Operator instructions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Retrieve,
    Hold,
    ReturnHome,
}

/// Coarse flight phase, shared by all missions (used for display and PX4 `nav_state` logging).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Phase {
    #[default]
    Idle,
    Search,
    Chase,
    Return,
    Land,
    Done,
}

impl Phase {
    pub fn label(self) -> &'static str {
        match self {
            Phase::Idle => "STANDING BY",
            Phase::Search => "SEARCHING",
            Phase::Chase => "PURSUIT",
            Phase::Return => "RETURNING TO BASE",
            Phase::Land => "LANDING",
            Phase::Done => "MISSION COMPLETE",
        }
    }
}

/// What a mission asks of guidance this step.
#[derive(Clone, Copy, Debug, Default)]
pub struct Objective {
    /// Where to fly (None = hover in place).
    pub goal: Option<Vec2>,
    /// Altitude to hold instead of cruise altitude.
    pub altitude: Option<f32>,
    /// Come to a stop at the goal rather than flying through it.
    pub stop: bool,
    /// Descend once over the goal.
    pub land: bool,
}

/// A radio message. `level` follows syslog / MAVLink severity (6 = info, 4 = warning).
#[derive(Clone, Debug)]
pub struct RadioMsg {
    pub time: f32,
    pub level: u8,
    pub text: String,
}

/// The radio log missions talk through.
#[derive(Default)]
pub struct Radio {
    pub messages: Vec<RadioMsg>,
    pub now: f32,
}

impl Radio {
    pub fn say(&mut self, text: &str) {
        self.messages.push(RadioMsg { time: self.now, level: 6, text: text.to_string() });
    }

    pub fn warn(&mut self, text: &str) {
        self.messages.push(RadioMsg { time: self.now, level: 4, text: text.to_string() });
    }
}

/// What a mission can see when deciding.
pub struct Ctx<'a> {
    pub time: f32,
    pub pos: Vec3,
    pub home: Vec3,
    pub map: &'a Grid,
    pub payload: Payload,
    pub track: Option<Track>,
    pub radio: &'a mut Radio,
}

impl Ctx<'_> {
    pub fn sees_beacon(&self) -> bool {
        self.track.is_some_and(|t| self.time - t.seen_at < 0.3)
    }
}

pub trait Mission {
    fn phase(&self) -> Phase;
    /// An operator instruction; acknowledge it on the radio.
    fn command(&mut self, cmd: Command, ctx: &mut Ctx);
    /// React to what was just perceived. Runs every step, even while a human flies.
    fn observe(&mut self, dt: f32, ctx: &mut Ctx);
    /// Decide where to go. Runs every step while the autopilot flies.
    fn objective(&mut self, dt: f32, ctx: &mut Ctx) -> Objective;
}

// ----------------------------------------------------------------------------------------------

const MIN_FRONTIER_DIST: f32 = 6.0; // ignore frontiers right next to the drone (m)
const GOAL_TIMEOUT: f32 = 15.0; // give up on an exploration goal after this long (s)
const RECHECK_EVERY: f32 = 1.0; // how often to check the goal is still a frontier (s)
const APPROACH: f32 = 12.0; // within this range, match the beacon's altitude (m)
const LOST_AFTER: f32 = 1.5; // s without a sighting before we call it "lost visual"
const LEAD_SPEED: f32 = 7.0; // speed assumed when leading the target (m/s)
const OVER_PAD: f32 = 2.0; // m
const TOUCHDOWN: f32 = 0.4; // m, landed below this height

/// Find the purple beacon in an unknown city, chase it down, and bring it home.
///
/// `Idle -> Search -> Chase -> Return -> Land -> Done`
pub struct Retrieve {
    phase: Phase,
    search: Option<SearchMap>,
    explore_goal: Option<Vec2>,
    goal_timer: f32,
    recheck_timer: f32,
    failed_searches: u32,
}

impl Default for Retrieve {
    fn default() -> Self {
        Self {
            phase: Phase::Idle,
            search: None,
            explore_goal: None,
            goal_timer: 0.0,
            recheck_timer: 0.0,
            failed_searches: 0,
        }
    }
}

impl Mission for Retrieve {
    fn phase(&self) -> Phase {
        self.phase
    }

    fn command(&mut self, cmd: Command, ctx: &mut Ctx) {
        match cmd {
            Command::Retrieve if self.phase == Phase::Done => ctx.radio.say("BEACON IS ALREADY HOME."),
            Command::Retrieve => {
                self.phase = if ctx.sees_beacon() { Phase::Chase } else { Phase::Search };
                self.explore_goal = None;
                ctx.radio.say("COPY. RETRIEVE THE PURPLE BEACON. MAP UNKNOWN, STARTING SEARCH.");
            }
            Command::Hold => {
                self.phase = Phase::Idle;
                ctx.radio.say("COPY. HOLDING POSITION.");
            }
            Command::ReturnHome => {
                self.phase = Phase::Return;
                ctx.radio.say("COPY. RETURNING TO BASE.");
            }
        }
    }

    fn observe(&mut self, dt: f32, ctx: &mut Ctx) {
        let half_extent = ctx.map.size() as f32 / 2.0; // the map uses 1 m cells
        self.search.get_or_insert_with(|| SearchMap::new(half_extent)).observe(dt, ctx.map, ctx.pos, ctx.time);
        let over_pad = vec2(ctx.pos.x, ctx.pos.z).distance(vec2(ctx.home.x, ctx.home.z)) < OVER_PAD;
        match (self.phase, ctx.payload) {
            (Phase::Search | Phase::Chase, Payload::Carrying) => {
                self.phase = Phase::Return;
                ctx.radio.say("GOT IT. BEACON SECURED. HEADING HOME.");
            }
            (Phase::Return | Phase::Land, Payload::Delivered) => {
                self.phase = Phase::Done;
                ctx.radio.say("BEACON DELIVERED. GOOD NIGHT.");
            }
            (Phase::Land, _) if ctx.pos.y < TOUCHDOWN => {
                self.phase = Phase::Done; // landed without the beacon: disarm
                ctx.radio.warn("LANDED. NO BEACON ON BOARD.");
            }
            (Phase::Search, _) if ctx.sees_beacon() => {
                self.phase = Phase::Chase;
                let b = ctx.track.unwrap().pos;
                ctx.radio.say(&format!("VISUAL ON THE BEACON AT {:.0}, {:.0}. IN PURSUIT.", b.x, b.z));
            }
            (Phase::Return, _) if over_pad => {
                self.phase = Phase::Land;
                ctx.radio.say("OVER THE PAD. SETTING DOWN.");
            }
            _ => {}
        }
    }

    fn objective(&mut self, dt: f32, ctx: &mut Ctx) -> Objective {
        let p2 = vec2(ctx.pos.x, ctx.pos.z);
        let home = vec2(ctx.home.x, ctx.home.z);
        match self.phase {
            Phase::Idle | Phase::Done => Objective::default(),
            Phase::Return => Objective { goal: Some(home), stop: true, ..Default::default() },
            Phase::Land => Objective { goal: Some(home), stop: true, land: true, ..Default::default() },
            Phase::Search => self.explore(dt, p2, ctx),
            Phase::Chase => {
                let t = ctx.track.expect("chasing requires a track");
                let age = ctx.time - t.seen_at;
                // Aim where the beacon will be when we get there (lead pursuit)
                let lead = if age < LOST_AFTER { (t.pos.distance(ctx.pos) / LEAD_SPEED).min(2.0) } else { 0.0 };
                let predicted = t.pos + t.vel * (age.min(4.0) + lead);
                let target = vec2(predicted.x, predicted.z);
                if age > LOST_AFTER && p2.distance(target) < 3.0 {
                    // We reached where it should be, and it isn't here
                    self.phase = Phase::Search;
                    self.explore_goal = None;
                    ctx.radio.warn("LOST VISUAL IN THE RAIN. RESUMING SEARCH.");
                    return self.explore(dt, p2, ctx);
                }
                let close = t.pos.distance(ctx.pos) < APPROACH && age < LOST_AFTER;
                Objective { goal: Some(target), altitude: close.then_some(predicted.y), ..Default::default() }
            }
        }
    }
}

impl Retrieve {
    /// Frontier exploration: keep a goal until it's reached, seen, or taking too long.
    fn explore(&mut self, dt: f32, p2: Vec2, ctx: &mut Ctx) -> Objective {
        self.goal_timer -= dt;
        self.recheck_timer -= dt;
        // Commit to a goal: re-check that it still needs looking at only once a second, otherwise
        // the sensors "see" the goal almost immediately and the drone dithers between goals.
        let recheck = self.recheck_timer <= 0.0;
        if recheck {
            self.recheck_timer = RECHECK_EVERY;
        }
        let search = self.search.as_ref().expect("observe() runs before objective()");
        let stale = self.explore_goal.is_none_or(|g| {
            g.distance(p2) < 3.0
                || self.goal_timer <= 0.0
                || (recheck && !ctx.map.is_frontier_at(g) && search.age(g, ctx.time) < 2.0)
        });
        if stale {
            let frontier = explore::nearest_frontier(ctx.map, p2, MIN_FRONTIER_DIST);
            self.explore_goal = search.best_goal(ctx.map, p2, ctx.time, frontier);
            self.goal_timer = GOAL_TIMEOUT;
            if self.explore_goal.is_none() {
                // Nothing reachable left to explore (checked a few times, to be sure)
                self.failed_searches += 1;
                if self.failed_searches < 3 {
                    self.goal_timer = RECHECK_EVERY;
                    return Objective::default(); // hover and look again
                }
                self.phase = Phase::Return;
                ctx.radio.warn("WHOLE SECTOR MAPPED. NO SIGN OF THE BEACON. COMING HOME.");
                return Objective { goal: Some(vec2(ctx.home.x, ctx.home.z)), stop: true, ..Default::default() };
            }
            self.failed_searches = 0;
        }
        Objective { goal: self.explore_goal, ..Default::default() }
    }
}
