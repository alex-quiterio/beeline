//! Flight recorder ("black box") writing **PX4 ULog** (`.ulg`), the log format real PX4 drones use.
//!
//! The file opens in standard tools: PX4 Flight Review (<https://logs.px4.io>), PlotJuggler, and
//! `pyulog` (`ulog_info`, `ulog_messages`, `ulog2csv`). Format spec:
//! <https://docs.px4.io/main/en/dev_log/ulog_file_format.html>
//!
//! * Standard uORB topics with PX4 field names, in the **NED** frame (x north, y east, z down),
//!   timestamps in microseconds since boot: `vehicle_local_position`, `vehicle_attitude`,
//!   `vehicle_local_position_setpoint`, `vehicle_status` (PX4 `nav_state` codes), `wind`.
//! * Controller settings are recorded as PX4 parameters (`MPC_XY_CRUISE`, `MPC_TILTMAX_AIR`, ...).
//! * Radio traffic, mode changes and collisions are ULog logged strings with syslog severities.
//! * Beeline-specific data uses its own `beeline_*` topics, the way PX4 forks add custom messages.

use crate::flight::{KI, KP, SETPOINT_ACCEL, Setpoint, YawCmd};
use crate::nav::mapping::Scan;
use crate::nav::{Autopilot, Phase};
use crate::sensors::Sensors;
use crate::sim::World;
use crate::vehicle::{DRAG, MAX_TILT};
use glam::Vec3;
use std::io;
use std::path::{Path, PathBuf};

const STATUS_EVERY: f32 = 0.2; // s, rate for slow topics (status, wind, mission)

/// (topic, ULog format). Message ids are the indices into this table.
const TOPICS: [(&str, &str); 8] = [
    ("vehicle_local_position", "uint64_t timestamp;float x;float y;float z;float vx;float vy;float vz;float heading;"),
    ("vehicle_attitude", "uint64_t timestamp;float[4] q;"),
    (
        "vehicle_local_position_setpoint",
        "uint64_t timestamp;float x;float y;float z;float vx;float vy;float vz;float yaw;",
    ),
    ("vehicle_status", "uint64_t timestamp;uint8_t nav_state;uint8_t arming_state;"),
    ("wind", "uint64_t timestamp;float windspeed_north;float windspeed_east;"),
    (
        "beeline_mission",
        "uint64_t timestamp;float target_x;float target_y;float target_z;float goal_x;float goal_y;float explored;float flow_left;float flow_right;uint16_t collisions;uint8_t phase;uint8_t avoiding;uint8_t payload;",
    ),
    (
        "beeline_ground_truth",
        "uint64_t timestamp;float beacon_x;float beacon_y;float beacon_z;float wind_north;float wind_east;",
    ),
    // PX4's standard 360° range message (as used by Collision Prevention): 72 sectors of 5°,
    // centimetres, index 0 = north, clockwise. max_distance + 1 means "nothing there".
    (
        "obstacle_distance",
        "uint64_t timestamp;uint8_t frame;uint8_t sensor_type;uint16_t[72] distances;float increment;uint16_t min_distance;uint16_t max_distance;float angle_offset;",
    ),
];
pub const OBSTACLE_SECTORS: usize = 72;
pub const OBSTACLE_MAX_CM: u16 = 2500;
const MAV_FRAME_LOCAL_NED: u8 = 1;
const MAV_DISTANCE_SENSOR_LASER: u8 = 0;
const POSITION: u16 = 0;
const ATTITUDE: u16 = 1;
const SETPOINT: u16 = 2;
const STATUS: u16 = 3;
const WIND: u16 = 4;
const MISSION: u16 = 5;
const TRUTH: u16 = 6;
const OBSTACLES: u16 = 7;

// PX4 vehicle_status.nav_state values
const NAV_POSCTL: u8 = 2;
const NAV_AUTO_MISSION: u8 = 3;
const NAV_AUTO_LOITER: u8 = 4;
const NAV_AUTO_RTL: u8 = 5;
const NAV_AUTO_LAND: u8 = 18;
const ARMING_STATE_ARMED: u8 = 2;

// Severities (syslog / MAVLink)
const WARNING: u8 = 4;
const INFO: u8 = 6;

pub struct FlightRecorder {
    buf: Vec<u8>,
    name: String,
    next_slow: f32,
    radio_seen: usize,
    collisions_seen: usize,
    last_auto: Option<bool>,
    last_scan_time: f32,
}

impl FlightRecorder {
    pub fn new(world: &World) -> Self {
        let name = format!("beeline_{}_{}", world.map.name().to_lowercase().replace(' ', "_"), world.seed);
        let mut r = Self {
            buf: Vec::with_capacity(1 << 20),
            name,
            next_slow: 0.0,
            radio_seen: 0,
            collisions_seen: 0,
            last_auto: None,
            last_scan_time: -1.0,
        };

        // File header: magic, version, start timestamp
        r.buf.extend_from_slice(b"ULog\x01\x12\x35");
        r.buf.push(1);
        r.buf.extend_from_slice(&0u64.to_le_bytes());
        r.message(b'B', &[0u8; 40]); // flag bits: no compat/incompat flags, no appended data

        // Definitions
        for (topic, fields) in TOPICS {
            r.message(b'F', format!("{topic}:{fields}").as_bytes());
        }
        r.info("sys_name", "Beeline");
        r.info("ver_hw", "BEELINE_SITL");
        r.info("ver_sw_release_str", env!("CARGO_PKG_VERSION"));
        r.info("sys_os_name", "Beeline simulator");
        r.info("beeline_map", world.map.name());
        r.info("beeline_seed", &world.seed.to_string());
        // Controller settings, under the PX4 parameter names they correspond to
        r.param("MPC_XY_CRUISE", 7.0);
        r.param("MPC_TILTMAX_AIR", MAX_TILT.to_degrees());
        r.param("MPC_ACC_HOR", SETPOINT_ACCEL);
        r.param("MPC_XY_VEL_P_ACC", KP.x);
        r.param("MPC_XY_VEL_I_ACC", KI.x);
        r.param("MPC_Z_VEL_P_ACC", KP.y);
        r.param("MPC_Z_VEL_I_ACC", KI.y);
        r.param("CP_DIST", 1.5);

        // Data section: subscribe to every topic
        for (id, (topic, _)) in TOPICS.iter().enumerate() {
            let mut p = vec![0u8];
            p.extend_from_slice(&(id as u16).to_le_bytes());
            p.extend_from_slice(topic.as_bytes());
            r.message(b'A', &p);
        }
        r
    }

    /// Record one control step. Call after `world.step()`.
    pub fn record(&mut self, world: &World, ap: &Autopilot, setpoint: &Setpoint, auto: bool, wind_estimate: Vec3) {
        let t = world.time;
        let s = world.drone_state();
        let (yaw, pitch, roll) = world.attitude();
        let heading = wrap(-yaw);

        let mut d = Data::default();
        d.vec3(ned(s.pos)).vec3(ned(s.vel)).f32(heading);
        self.data(POSITION, t, d);

        let mut d = Data::default();
        for q in quaternion(-roll, pitch, heading) {
            d.f32(q);
        }
        self.data(ATTITUDE, t, d);

        let sp_yaw = match setpoint.yaw {
            YawCmd::Heading(h) => wrap(-h),
            YawCmd::Rate(_) => f32::NAN,
        };
        let mut d = Data::default();
        d.vec3([f32::NAN; 3]).vec3(ned(setpoint.vel)).f32(sp_yaw);
        self.data(SETPOINT, t, d);

        if t >= self.next_slow {
            self.next_slow = t + STATUS_EVERY;
            let mut d = Data::default();
            d.u8(nav_state(auto, ap.phase())).u8(ARMING_STATE_ARMED);
            self.data(STATUS, t, d);

            // Wind: the controller's estimate, converted from acceleration to air speed via drag
            let w = ned(wind_estimate / DRAG);
            let mut d = Data::default();
            d.f32(w[0]).f32(w[1]);
            self.data(WIND, t, d);

            let target = ap.track.map_or([f32::NAN; 3], |tr| ned(tr.pos));
            let goal = ap.goal.map_or([f32::NAN; 2], |g| {
                let n = ned(Vec3::new(g.x, 0.0, g.y));
                [n[0], n[1]]
            });
            let mut d = Data::default();
            d.vec3(target)
                .f32(goal[0])
                .f32(goal[1])
                .f32(ap.map.explored())
                .f32(ap.flow.left)
                .f32(ap.flow.right)
                .u16(world.collisions.len() as u16)
                .u8(ap.phase() as u8)
                .u8(ap.avoiding as u8)
                .u8(world.payload() as u8);
            self.data(MISSION, t, d);

            let w = ned(world.wind() / DRAG);
            let mut d = Data::default();
            d.vec3(ned(world.beacon.pos)).f32(w[0]).f32(w[1]);
            self.data(TRUTH, t, d);
        }

        // Lidar: one obstacle_distance message per mapping scan
        if let Some(scan) = &ap.last_scan
            && scan.time > self.last_scan_time
        {
            self.last_scan_time = scan.time;
            let sectors = obstacle_sectors(scan);
            let mut d = Data::default();
            d.u8(MAV_FRAME_LOCAL_NED).u8(MAV_DISTANCE_SENSOR_LASER);
            for cm in sectors {
                d.u16(cm);
            }
            d.f32(360.0 / OBSTACLE_SECTORS as f32).u16(20).u16(OBSTACLE_MAX_CM).f32(0.0);
            self.data(OBSTACLES, scan.time, d);
        }

        // Events
        if self.last_auto != Some(auto) {
            self.last_auto = Some(auto);
            self.log(INFO, t, if auto { "Mode: AUTO" } else { "Mode: POSITION (manual)" });
        }
        for msg in &ap.radio()[self.radio_seen..] {
            let line = format!("[radio] {}", msg.text);
            log_string(&mut self.buf, msg.level, msg.time, &line);
        }
        self.radio_seen = ap.radio().len();
        for c in &world.collisions[self.collisions_seen..] {
            let what = c.kind.map_or("unknown".to_string(), |k| format!("{k:?}"));
            log_string(&mut self.buf, WARNING, c.time, &format!("Collision with {what}"));
        }
        self.collisions_seen = world.collisions.len();
    }

    pub fn bytes(&self) -> &[u8] {
        &self.buf
    }

    /// Write `<dir>/<name>_<unix time>.ulg` and return the path.
    pub fn save(&self, dir: &Path) -> io::Result<PathBuf> {
        std::fs::create_dir_all(dir)?;
        let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
        let path = dir.join(format!("{}_{stamp}.ulg", self.name));
        std::fs::write(&path, &self.buf)?;
        Ok(path)
    }

    fn message(&mut self, kind: u8, payload: &[u8]) {
        message(&mut self.buf, kind, payload);
    }

    fn info(&mut self, key: &str, value: &str) {
        let key = format!("char[{}] {key}", value.len());
        let mut p = vec![key.len() as u8];
        p.extend_from_slice(key.as_bytes());
        p.extend_from_slice(value.as_bytes());
        self.message(b'I', &p);
    }

    fn param(&mut self, name: &str, value: f32) {
        let key = format!("float {name}");
        let mut p = vec![key.len() as u8];
        p.extend_from_slice(key.as_bytes());
        p.extend_from_slice(&value.to_le_bytes());
        self.message(b'P', &p);
    }

    fn data(&mut self, id: u16, t: f32, d: Data) {
        let mut p = Vec::with_capacity(10 + d.0.len());
        p.extend_from_slice(&id.to_le_bytes());
        p.extend_from_slice(&micros(t).to_le_bytes());
        p.extend_from_slice(&d.0);
        self.message(b'D', &p);
    }

    fn log(&mut self, level: u8, t: f32, text: &str) {
        log_string(&mut self.buf, level, t, text);
    }
}

fn message(buf: &mut Vec<u8>, kind: u8, payload: &[u8]) {
    buf.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    buf.push(kind);
    buf.extend_from_slice(payload);
}

/// ULog 'L' message: severity as an ASCII digit, as PX4 writes it.
fn log_string(buf: &mut Vec<u8>, level: u8, t: f32, text: &str) {
    let mut p = vec![b'0' + level];
    p.extend_from_slice(&micros(t).to_le_bytes());
    p.extend_from_slice(text.as_bytes());
    message(buf, b'L', &p);
}

/// Little-endian field packer for a data message.
#[derive(Default)]
struct Data(Vec<u8>);

impl Data {
    fn f32(&mut self, v: f32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn vec3(&mut self, v: [f32; 3]) -> &mut Self {
        self.f32(v[0]).f32(v[1]).f32(v[2])
    }
    fn u16(&mut self, v: u16) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn u8(&mut self, v: u8) -> &mut Self {
        self.0.push(v);
        self
    }
}

fn micros(t: f32) -> u64 {
    (t as f64 * 1e6) as u64
}

/// Simulator frame (x east, y up, z south) -> NED (north, east, down).
fn ned(v: Vec3) -> [f32; 3] {
    [-v.z, v.x, -v.y]
}

/// NED body-to-earth quaternion (w, x, y, z) from roll, pitch, yaw.
fn quaternion(roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sy, cy) = (yaw / 2.0).sin_cos();
    [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
}

/// Bin a mapping scan into PX4 obstacle_distance sectors (closest return per sector, in cm).
fn obstacle_sectors(scan: &Scan) -> [u16; OBSTACLE_SECTORS] {
    let mut out = [OBSTACLE_MAX_CM + 1; OBSTACLE_SECTORS];
    for (i, range) in scan.ranges.iter().enumerate() {
        let Some(r) = range else { continue };
        let a = i as f32 / scan.ranges.len() as f32 * std::f32::consts::TAU;
        let d = ned(Vec3::new(a.cos(), 0.0, a.sin()));
        let bearing = d[1].atan2(d[0]).to_degrees().rem_euclid(360.0); // from north, clockwise
        let sector = (bearing / (360.0 / OBSTACLE_SECTORS as f32)) as usize % OBSTACLE_SECTORS;
        out[sector] = out[sector].min((r * 100.0).round().min(OBSTACLE_MAX_CM as f32) as u16);
    }
    out
}

fn wrap(a: f32) -> f32 {
    crate::flight::wrap_angle(a)
}

fn nav_state(auto: bool, phase: Phase) -> u8 {
    if !auto {
        return NAV_POSCTL;
    }
    match phase {
        Phase::Idle | Phase::Done => NAV_AUTO_LOITER,
        Phase::Search | Phase::Chase => NAV_AUTO_MISSION,
        Phase::Return => NAV_AUTO_RTL,
        Phase::Land => NAV_AUTO_LAND,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ned_and_attitude_conventions() {
        // Our forward (-z) is north; up is -down
        assert_eq!(ned(Vec3::new(0.0, 0.0, -1.0)), [1.0, 0.0, -0.0]);
        assert_eq!(ned(Vec3::new(0.0, 1.0, 0.0)), [-0.0, 0.0, -1.0]);
        let q = quaternion(0.0, 0.0, 0.0);
        assert!((q[0] - 1.0).abs() < 1e-6);
    }
}
