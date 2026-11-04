//! Fleet intel: learn the terrain from other drones' flight logs before taking off.
//!
//! Every Beeline flight writes a PX4 ULog containing `vehicle_local_position` and
//! `obstacle_distance` (360° range readings). Replaying those through **log-odds occupancy
//! mapping** rebuilds the terrain those drones saw:
//!
//! * a ray passing through a cell is evidence it's empty (`MISS`), a ray ending there is evidence
//!   of an obstacle (`HIT`); evidence adds up across scans and across flights;
//! * walls get hit over and over, while moving things (gates, patrols) are mostly seen *through*,
//!   so they fade out instead of becoming ghost walls.
//!
//! The ULog reader parses the field layouts stored in the file itself, so it reads any log that
//! has those two standard topics, not just Beeline's.

use crate::nav::mapping::MARGIN;
use crate::nav::planner::Grid;
use glam::{Vec2, vec2};
use std::collections::HashMap;
use std::path::Path;

const CELL: f32 = 1.0;
const HIT: f32 = 0.85;
const MISS: f32 = -0.4;
const LIMIT: f32 = 4.0; // clamp, so old evidence can still be overturned
const WALL: f32 = 1.6; // log-odds above this counts as a wall

/// Minimal ULog reader: info strings, message formats, subscriptions and data, in file order.
pub struct Ulog {
    pub info: HashMap<String, String>,
    formats: HashMap<String, Vec<(String, String)>>, // topic -> [(type, name)]
    topics: HashMap<u16, String>,                    // msg_id -> topic
    data: Vec<(u16, Vec<u8>)>,                       // (msg_id, struct bytes)
}

impl Ulog {
    pub fn parse(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < 16 || &bytes[..7] != b"ULog\x01\x12\x35" {
            return Err("not a ULog file".into());
        }
        let mut log = Self { info: HashMap::new(), formats: HashMap::new(), topics: HashMap::new(), data: vec![] };
        let mut pos = 16;
        while pos + 3 <= bytes.len() {
            let size = u16::from_le_bytes([bytes[pos], bytes[pos + 1]]) as usize;
            let kind = bytes[pos + 2];
            let Some(p) = bytes.get(pos + 3..pos + 3 + size) else { break }; // truncated tail
            pos += 3 + size;
            match kind {
                b'F' => {
                    let text = String::from_utf8_lossy(p);
                    if let Some((name, fields)) = text.split_once(':') {
                        let fields = fields
                            .split(';')
                            .filter(|f| !f.is_empty())
                            .filter_map(|f| f.split_once(' ').map(|(t, n)| (t.to_string(), n.to_string())))
                            .collect();
                        log.formats.insert(name.to_string(), fields);
                    }
                }
                b'I' if !p.is_empty() => {
                    let key_len = p[0] as usize;
                    let key = String::from_utf8_lossy(&p[1..(1 + key_len).min(p.len())]);
                    if let Some((ty, name)) = key.split_once(' ')
                        && ty.starts_with("char[")
                    {
                        let value = String::from_utf8_lossy(&p[(1 + key_len).min(p.len())..]);
                        log.info.insert(name.to_string(), value.to_string());
                    }
                }
                b'A' if p.len() >= 3 => {
                    let id = u16::from_le_bytes([p[1], p[2]]);
                    log.topics.insert(id, String::from_utf8_lossy(&p[3..]).to_string());
                }
                b'D' if p.len() >= 2 => log.data.push((u16::from_le_bytes([p[0], p[1]]), p[2..].to_vec())),
                _ => {}
            }
        }
        Ok(log)
    }

    /// Byte offset of `field` within `topic`'s struct.
    fn offset(&self, topic: &str, field: &str) -> Option<usize> {
        let mut off = 0;
        for (ty, name) in self.formats.get(topic)? {
            if name == field {
                return Some(off);
            }
            off += self.size_of(ty)?;
        }
        None
    }

    fn size_of(&self, ty: &str) -> Option<usize> {
        if let Some((base, n)) = ty.strip_suffix(']').and_then(|t| t.split_once('[')) {
            return Some(self.size_of(base)? * n.parse::<usize>().ok()?);
        }
        Some(match ty {
            "int8_t" | "uint8_t" | "bool" | "char" => 1,
            "int16_t" | "uint16_t" => 2,
            "int32_t" | "uint32_t" | "float" => 4,
            "int64_t" | "uint64_t" | "double" => 8,
            nested => self.formats.get(nested)?.iter().map(|(t, _)| self.size_of(t)).sum::<Option<usize>>()?,
        })
    }
}

fn f32_at(b: &[u8], off: usize) -> Option<f32> {
    Some(f32::from_le_bytes(b.get(off..off + 4)?.try_into().ok()?))
}

fn u16_at(b: &[u8], off: usize) -> Option<u16> {
    Some(u16::from_le_bytes(b.get(off..off + 2)?.try_into().ok()?))
}

/// Accumulated evidence about the terrain, from any number of flight logs.
pub struct TerrainPrior {
    half_extent: f32,
    size: i32,
    logodds: Vec<f32>,
    seen: Vec<bool>,
    pub flights: usize,
    pub scans: usize,
}

impl TerrainPrior {
    pub fn new(half_extent: f32) -> Self {
        let size = (half_extent * 2.0 / CELL).ceil() as i32;
        let n = (size * size) as usize;
        Self { half_extent, size, logodds: vec![0.0; n], seen: vec![false; n], flights: 0, scans: 0 }
    }

    /// Replay one flight log's position + obstacle_distance messages. Returns the scans used.
    pub fn absorb(&mut self, log: &Ulog) -> usize {
        let topic_of = |id: u16| log.topics.get(&id).map(String::as_str);
        let (Some(px), Some(py)) =
            (log.offset("vehicle_local_position", "x"), log.offset("vehicle_local_position", "y"))
        else {
            return 0;
        };
        let get = |f: &str| log.offset("obstacle_distance", f);
        let (Some(dist), Some(inc), Some(maxd), Some(offs)) =
            (get("distances"), get("increment"), get("max_distance"), get("angle_offset"))
        else {
            return 0;
        };
        let sectors = log.formats["obstacle_distance"]
            .iter()
            .find(|(_, n)| n == "distances")
            .and_then(|(t, _)| t.strip_suffix(']')?.split_once('[')?.1.parse::<usize>().ok())
            .unwrap_or(72);

        let mut here: Option<Vec2> = None; // (north, east)
        let mut used = 0;
        for (id, b) in &log.data {
            match topic_of(*id) {
                Some("vehicle_local_position") => here = f32_at(b, px).zip(f32_at(b, py)).map(|(n, e)| vec2(n, e)),
                Some("obstacle_distance") => {
                    let (Some(p), Some(inc), Some(maxd), Some(offs)) =
                        (here, f32_at(b, inc), u16_at(b, maxd), f32_at(b, offs))
                    else {
                        continue;
                    };
                    for s in 0..sectors {
                        let Some(cm) = u16_at(b, dist + 2 * s) else { break };
                        if cm == u16::MAX {
                            continue;
                        } // unknown sector
                        let bearing = (offs + (s as f32 + 0.5) * inc).to_radians(); // from north, clockwise
                        let hit = cm <= maxd;
                        let range = cm.min(maxd) as f32 / 100.0;
                        self.ray(p, vec2(bearing.cos(), bearing.sin()), range, hit);
                    }
                    used += 1;
                }
                _ => {}
            }
        }
        if used > 0 {
            self.flights += 1;
            self.scans += used;
        }
        used
    }

    /// One range reading from `from` (north, east) along `dir`.
    fn ray(&mut self, from: Vec2, dir: Vec2, range: f32, hit: bool) {
        let mut last = None;
        let steps = (range / (CELL * 0.5)) as i32;
        for i in 0..steps {
            let c = self.cell(from + dir * (i as f32 * CELL * 0.5));
            if c != last
                && let Some(i) = c
            {
                self.seen[i] = true;
                self.logodds[i] = (self.logodds[i] + MISS).max(-LIMIT);
            }
            last = c;
        }
        if hit && let Some(i) = self.cell(from + dir * (range + 0.3)) {
            self.seen[i] = true;
            self.logodds[i] = (self.logodds[i] + HIT).min(LIMIT);
        }
    }

    /// Cell index of a NED (north, east) point, in the same layout as the autopilot's map.
    fn cell(&self, ne: Vec2) -> Option<usize> {
        let sim = vec2(ne.y, -ne.x); // (x = east, z = south)
        let c = ((sim + self.half_extent) / CELL).floor();
        let (x, y) = (c.x as i32, c.y as i32);
        (x >= 0 && y >= 0 && x < self.size && y < self.size).then(|| (y * self.size + x) as usize)
    }

    /// The autopilot map this evidence implies: seen cells are known, confident hits are walls.
    pub fn to_map(&self) -> Grid {
        let mut map = Grid::new(self.half_extent, CELL);
        for i in 0..self.logodds.len() {
            let c = ((i as i32) % self.size, (i as i32) / self.size);
            if self.seen[i] {
                map.mark_known(c);
            }
            if self.logodds[i] > WALL {
                map.mark_wall(map.center_of(c), MARGIN);
            }
        }
        map
    }
}

/// Every ULog in `dir` recorded over the same city (map + seed), merged into one prior map.
pub fn load_prior(dir: &Path, map_name: &str, seed: u64, half_extent: f32) -> Option<(Grid, usize)> {
    let mut prior = TerrainPrior::new(half_extent);
    for path in matching_logs(dir, map_name, seed) {
        if let Ok(log) = std::fs::read(&path).map_err(|e| e.to_string()).and_then(|b| Ulog::parse(&b)) {
            prior.absorb(&log);
        }
    }
    (prior.flights > 0).then(|| (prior.to_map(), prior.flights))
}

/// Paths of the logs in `dir` recorded over this map + seed.
pub fn matching_logs(dir: &Path, map_name: &str, seed: u64) -> Vec<std::path::PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else { return vec![] };
    let mut paths: Vec<_> =
        entries.flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "ulg")).collect();
    paths.sort();
    paths
        .into_iter()
        .filter(|p| {
            let Ok(bytes) = std::fs::read(p) else { return false };
            Ulog::parse(&bytes).is_ok_and(|l| {
                l.info.get("beeline_map").is_some_and(|m| m == map_name)
                    && l.info.get("beeline_seed").is_some_and(|s| *s == seed.to_string())
            })
        })
        .collect()
}
