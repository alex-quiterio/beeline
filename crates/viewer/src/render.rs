//! Noir viewer: black-and-white night city, fog, rain driven by the wind, film grain and vignette.
//! Two colours only: red marks the drone and its intent, purple marks the beacon.
//! Pure presentation; it only reads the world and the autopilot.

use beeline::flight::{FlightController, wrap_angle};
use beeline::nav::planner::CellState;
use beeline::nav::{Autopilot, Phase, clock};
use beeline::sensors::Sensors;
use beeline::sim::beacon::BeaconState;
use beeline::sim::city::{Kind, MapKind, Shape};
use beeline::sim::{Pose, World};
use std::path::Path;

use macroquad::prelude::*;

const NIGHT: Color = Color::new(0.03, 0.03, 0.04, 1.0);
const BLOOD: Color = Color::new(0.82, 0.07, 0.07, 1.0);
const VIOLET: Color = Color::new(0.66, 0.3, 1.0, 1.0);
const LAMP: Color = Color::new(0.95, 0.85, 0.6, 1.0); // lit windows
const FOG_DIST: f32 = 130.0;
const MINIMAP: f32 = 230.0; // px

/// Session facts the HUD needs beyond the world and the autopilot.
pub struct Hud<'a> {
    pub auto: bool,
    pub fleet_flights: usize,
    pub toast: Option<&'a str>,
    pub log_path: Option<&'a Path>,
}

pub struct Renderer {
    cam: Vec3,
    cam_yaw: f32,
    wind_travel: Vec3,
    minimap: Option<(Texture2D, Image)>,
    minimap_refresh: f64,
}

impl Renderer {
    pub fn new() -> Self {
        Self { cam: vec3(0.0, 3.5, 6.0), cam_yaw: 0.0, wind_travel: Vec3::ZERO, minimap: None, minimap_refresh: 0.0 }
    }

    /// Forget per-session state (camera, minimap) before a new deployment.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    // ------------------------------------------------------------------ menu

    pub fn draw_menu(&mut self, selected: usize, seed: u64, fleet_logs: &[usize]) {
        clear_background(NIGHT);
        set_default_camera();
        let (sw, sh) = (screen_width(), screen_height());
        let t = get_time() as f32;

        // Rain on the glass
        for i in 0..260u32 {
            let x = hash(i, 1) * sw;
            let y = (hash(i, 2) * sh + t * (500.0 + hash(i, 3) * 300.0)) % sh;
            draw_line(x, y, x - 6.0, y + 18.0, 1.0, Color::new(0.7, 0.72, 0.8, 0.18));
        }

        let cx = sw / 2.0;
        centered("B E E L I N E", cx, sh * 0.22, 64.0, grey(0.92));
        centered("an autonomous drone in a city that never sleeps", cx, sh * 0.22 + 38.0, 22.0, grey(0.5));
        centered("OBJECTIVE: FIND AND RETRIEVE THE PURPLE BEACON. THE DRONE HAS NO MAP.", cx, sh * 0.36, 20.0, VIOLET);

        let (w, h, gap) = (360.0, 170.0, 40.0);
        for (i, map) in MapKind::ALL.iter().enumerate() {
            let x = cx - w - gap / 2.0 + i as f32 * (w + gap);
            let y = sh * 0.44;
            let on = i == selected;
            draw_rectangle(x, y, w, h, Color::new(0.06, 0.06, 0.07, 0.95));
            draw_rectangle_lines(x, y, w, h, if on { 3.0 } else { 1.0 }, if on { BLOOD } else { grey(0.3) });
            draw_text(
                format!("{}  {}", i + 1, map.name()),
                x + 20.0,
                y + 42.0,
                30.0,
                if on { grey(0.95) } else { grey(0.6) },
            );
            wrap_text(map.tagline(), x + 20.0, y + 78.0, 18.0, w - 40.0, grey(0.55));
            let intel = match fleet_logs.get(i).copied().unwrap_or(0) {
                0 => "NO FLEET INTEL: FLYING BLIND".to_string(),
                n => format!("FLEET INTEL: {n} EARLIER FLIGHT LOG(S)"),
            };
            draw_text(
                &intel,
                x + 20.0,
                y + h - 20.0,
                16.0,
                if fleet_logs.get(i).copied().unwrap_or(0) > 0 { BLOOD } else { grey(0.4) },
            );
        }
        centered(&format!("CITY SEED {seed}"), cx, sh * 0.44 + h + 50.0, 18.0, grey(0.45));
        centered("1 / 2 or ARROWS + ENTER  deploy      N  new city", cx, sh * 0.44 + h + 80.0, 18.0, grey(0.45));
        self.post_fx();
    }

    // ------------------------------------------------------------------ flight

    pub fn draw(&mut self, world: &World, ap: &Autopilot, fc: &FlightController, alpha: f32, dt: f32, hud: Hud) {
        let drone = world.drone_pose(alpha);

        // Chase camera, smoothed independently of frame rate
        self.cam_yaw += wrap_angle(world.yaw() - self.cam_yaw) * (1.0 - (-4.0 * dt).exp());
        let arm = Quat::from_rotation_y(self.cam_yaw) * vec3(0.0, 5.0, 9.0);
        // Spring arm: if a building is between the drone and the camera, pull the camera in front of it
        let reach =
            world.raycast(drone.pos, arm.normalize(), arm.length()).map_or(arm.length(), |h| (h.dist - 0.6).max(1.5));
        let wanted = drone.pos + arm.normalize() * reach;
        self.cam = self.cam.lerp(wanted, 1.0 - (-10.0 * dt).exp());
        self.wind_travel += world.wind() * dt;
        let cam = self.cam;
        let fog = |c: Color, p: Vec3| mix(c, NIGHT, (cam.distance(p) / FOG_DIST).clamp(0.0, 1.0).powf(0.7));

        clear_background(NIGHT);
        set_camera(&Camera3D { position: cam, target: drone.pos, up: Vec3::Y, ..Default::default() });

        let ext = world.city.half_extent;
        draw_plane(vec3(0.0, -0.01, 0.0), vec2(ext + 60.0, ext + 60.0), None, Color::new(0.06, 0.06, 0.07, 1.0));
        let lines = (ext * 2.0 / world.city.block).round() as u32;
        draw_grid(lines, world.city.block, Color::new(0.16, 0.16, 0.17, 1.0), Color::new(0.1, 0.1, 0.11, 1.0));
        if world.city.kind == MapKind::Docks {
            // Black water along the north quay
            draw_plane(vec3(0.0, 0.0, -ext - 40.0), vec2(ext + 60.0, 30.0), None, Color::new(0.02, 0.025, 0.035, 1.0));
        }

        // Home pad
        let home = world.home;
        let landing = ap.phase() == Phase::Land;
        draw_cube(home + Vec3::Y * 0.02, vec3(5.0, 0.04, 5.0), None, grey(0.1));
        draw_cube_wires(home + Vec3::Y * 0.02, vec3(5.0, 0.04, 5.0), if landing { BLOOD } else { grey(0.6) });
        for (a, b) in [((-0.8, -1.2), (-0.8, 1.2)), ((0.8, -1.2), (0.8, 1.2)), ((-0.8, 0.0), (0.8, 0.0))] {
            draw_line_3d(home + vec3(a.0, 0.06, a.1), home + vec3(b.0, 0.06, b.1), grey(0.7));
        }

        // City
        for (i, o) in world.city.obstacles.iter().enumerate() {
            let pose = world.obstacle_pose(i, alpha);
            let (body, edge) = match o.kind {
                Kind::Building { shade, .. } => (grey(0.08 + shade * 0.3), grey(0.55)),
                Kind::Container { shade } => (grey(0.08 + shade * 0.28), grey(0.5)),
                Kind::Crane => (grey(0.14), grey(0.5)),
                Kind::Gate => (grey(0.18), grey(0.45)),
                Kind::Lift => (grey(0.3), grey(0.6)),
                Kind::Fan => (grey(0.1), grey(0.35)),
                Kind::Patrol => (grey(0.85), grey(0.85)), // pale "searchlight" orbs
            };
            let (body, edge, lamp) = (fog(body, pose.pos), fog(edge, pose.pos), fog(LAMP, pose.pos));
            with_pose(pose, || match o.shape {
                Shape::Box(s) => {
                    draw_cube(Vec3::ZERO, s, None, body);
                    draw_cube_wires(Vec3::ZERO, s, edge);
                    if matches!(o.kind, Kind::Building { windows: true, .. }) {
                        draw_windows(i as u32, s, lamp);
                    }
                }
                Shape::Ball(r) => draw_sphere(Vec3::ZERO, r, None, body),
            });
        }

        // The beacon: a pulsing purple light with a faint column, so the viewer can find it
        let b = world.beacon_pos(alpha);
        let pulse = 0.5 + 0.5 * (get_time() as f32 * 4.0).sin();
        draw_sphere(b, 0.45, None, VIOLET);
        draw_sphere_wires(
            b,
            0.8 + pulse * 0.5,
            None,
            Color::new(VIOLET.r, VIOLET.g, VIOLET.b, 0.15 + 0.2 * (1.0 - pulse)),
        );
        match world.beacon.state {
            BeaconState::Free => draw_line_3d(
                vec3(b.x, 0.0, b.z),
                vec3(b.x, b.y + 30.0, b.z),
                Color::new(VIOLET.r, VIOLET.g, VIOLET.b, 0.12),
            ),
            BeaconState::Carried => draw_line_3d(drone.pos, b, grey(0.6)),
            BeaconState::Delivered => {}
        }

        // Intent: route and sensors while the autopilot flies
        if hud.auto {
            for w in ap.path.windows(2) {
                draw_line_3d(vec3(w[0].x, 0.1, w[0].y), vec3(w[1].x, 0.1, w[1].y), Color::new(0.82, 0.07, 0.07, 0.6));
            }
            for r in &ap.rays {
                let c = if r.close { Color::new(0.82, 0.07, 0.07, 0.7) } else { Color::new(1.0, 1.0, 1.0, 0.06) };
                draw_line_3d(r.from, r.to, c);
            }
        }

        // The drone: dark airframe, red light at the nose, white at the tail
        with_pose(drone, || {
            draw_cube(Vec3::ZERO, vec3(0.6, 0.15, 0.6), None, grey(0.12));
            draw_cube_wires(Vec3::ZERO, vec3(0.6, 0.15, 0.6), grey(0.5));
            for (x, z) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                draw_line_3d(Vec3::ZERO, vec3(x * 0.6, 0.0, z * 0.6), grey(0.4));
                draw_cube(vec3(x * 0.6, 0.08, z * 0.6), vec3(0.45, 0.02, 0.45), None, Color::new(0.7, 0.7, 0.7, 0.35));
            }
            draw_cube(vec3(0.0, 0.05, -0.32), vec3(0.12, 0.08, 0.06), None, BLOOD);
            draw_cube(vec3(0.0, 0.05, 0.32), vec3(0.1, 0.06, 0.06), None, WHITE);
        });

        self.draw_rain(world, drone.pos);
        set_default_camera();
        self.post_fx();
        self.draw_hud(world, ap, fc, &hud);
        self.draw_minimap(world, ap);
    }

    /// Rain falls through a 40 m box around the drone and slants with the wind, so you can see it.
    fn draw_rain(&self, world: &World, center: Vec3) {
        let w = world.wind();
        let streak = vec3(w.x * 0.25, -1.0, w.z * 0.25).normalize() * 0.9;
        let drift = self.wind_travel * 2.0 + vec3(0.0, -world.time * 16.0, 0.0);
        let half = Vec3::splat(20.0);
        for i in 0..500u32 {
            let seed = vec3(hash(i, 1), hash(i, 2), hash(i, 3)) * 40.0;
            let p = (seed + drift - center).rem_euclid(half * 2.0) + center - half;
            if p.y > 0.0 {
                draw_line_3d(p, p + streak, Color::new(0.65, 0.68, 0.75, 0.25));
            }
        }
    }

    /// Vignette and film grain.
    fn post_fx(&self) {
        let (sw, sh) = (screen_width(), screen_height());
        for i in 0..48 {
            let inset = i as f32 * 4.0;
            let a = 0.06 * (1.0 - i as f32 / 48.0).powi(2);
            draw_rectangle_lines(inset, inset, sw - 2.0 * inset, sh - 2.0 * inset, 4.0, Color::new(0.0, 0.0, 0.0, a));
        }
        for _ in 0..350 {
            let a = rand::gen_range(0.02, 0.09);
            draw_rectangle(rand::gen_range(0.0, sw), rand::gen_range(0.0, sh), 1.5, 1.5, Color::new(1.0, 1.0, 1.0, a));
        }
    }

    fn draw_hud(&self, world: &World, ap: &Autopilot, fc: &FlightController, hud: &Hud) {
        let (sw, sh) = (screen_width(), screen_height());
        let s = world.drone_state();

        draw_text("B E E L I N E", 24.0, 36.0, 28.0, grey(0.88));
        draw_text(format!("{}  ·  SEED {}", world.map.name(), world.seed), 24.0, 58.0, 18.0, grey(0.45));
        draw_text(
            format!(
                "ALT {:>5.1} m   SPD {:>4.1} m/s   WIND {:>3.1}   HITS {}   MAP {:>3.0}%",
                s.pos.y,
                s.vel.length(),
                world.wind().length(),
                world.collisions.len(),
                ap.map.explored() * 100.0
            ),
            24.0,
            84.0,
            19.0,
            grey(0.6),
        );

        let mode = if hud.auto { "AUTOPILOT" } else { "MANUAL" };
        let busy = !matches!(ap.phase(), Phase::Idle | Phase::Done) && hud.auto;
        draw_text(
            format!("{mode}  ·  {}", ap.phase().label()),
            24.0,
            112.0,
            24.0,
            if busy { BLOOD } else { grey(0.85) },
        );
        let mut y = 136.0;
        if let Some(t0) = ap.mission_start {
            let end = world.delivered_at.unwrap_or(ap.time);
            draw_text(format!("MISSION {}", clock(end - t0)), 24.0, y, 19.0, grey(0.6));
            y += 22.0;
        }
        if ap.sees_beacon() {
            draw_circle(30.0, y - 6.0, 5.0, VIOLET);
            draw_text("VISUAL ON BEACON", 42.0, y, 19.0, VIOLET);
            y += 22.0;
        }
        if hud.fleet_flights > 0 {
            draw_text(format!("FLEET INTEL · {} FLIGHT LOG(S)", hud.fleet_flights), 24.0, y, 17.0, grey(0.5));
        }

        // Radio log, newest at the bottom
        let radio = ap.recent_radio();
        for (i, m) in radio.iter().enumerate() {
            let fade = 0.35 + 0.6 * (i + 1) as f32 / radio.len() as f32;
            let c = if m.level <= 4 {
                Color::new(BLOOD.r, BLOOD.g, BLOOD.b, fade)
            } else {
                Color::new(0.85, 0.85, 0.85, fade)
            };
            let y = sh - 70.0 - (radio.len() - 1 - i) as f32 * 22.0;
            draw_text(format!("{}  {}", clock(m.time), m.text), 24.0, y, 18.0, c);
        }
        draw_text(
            "F retrieve beacon   H hold   B base   T manual/auto   ARROWS W S A D fly   L save log   R redeploy   M maps",
            24.0,
            sh - 24.0,
            16.0,
            grey(0.38),
        );

        // Bee "eyes": optic flow on each side (centring = keeping them balanced)
        if hud.auto && busy {
            let (cx, y) = (sw / 2.0, sh - 110.0);
            let (l, r) = ((ap.flow.left * 40.0).min(160.0), (ap.flow.right * 40.0).min(160.0));
            draw_rectangle(cx - 6.0 - l, y, l, 5.0, grey(0.7));
            draw_rectangle(cx + 6.0, y, r, 5.0, grey(0.7));
            draw_line(cx, y - 6.0, cx, y + 11.0, 1.0, BLOOD);
            centered("OPTIC FLOW", cx, y - 10.0, 15.0, grey(0.45));
        }

        // Wind compass under the minimap: true wind (white) vs the controller's estimate (red)
        let c = vec2(sw - 24.0 - MINIMAP / 2.0, 24.0 + MINIMAP + 60.0);
        let to_screen = |w: Vec3| {
            let r = Quat::from_rotation_y(-self.cam_yaw) * w;
            c + vec2(r.x, r.z) * 7.0
        };
        draw_circle_lines(c.x, c.y, 32.0, 1.0, grey(0.45));
        let (t, e) = (to_screen(world.wind()), to_screen(fc.wind_estimate()));
        draw_line(c.x, c.y, t.x, t.y, 2.0, grey(0.85));
        draw_line(c.x, c.y, e.x, e.y, 2.0, BLOOD);
        centered("WIND", c.x, c.y + 50.0, 15.0, grey(0.45));

        // Mission complete card
        if ap.phase() == Phase::Done {
            let (w, h) = (520.0, 120.0);
            let (x, y) = (sw / 2.0 - w / 2.0, sh * 0.3);
            draw_rectangle(x, y, w, h, Color::new(0.0, 0.0, 0.0, 0.75));
            draw_rectangle_lines(x, y, w, h, 2.0, VIOLET);
            centered("BEACON DELIVERED", sw / 2.0, y + 48.0, 40.0, grey(0.95));
            let t = world.delivered_at.unwrap_or(0.0) - ap.mission_start.unwrap_or(0.0);
            let log = hud
                .log_path
                .and_then(|p| p.file_name())
                .map_or(String::new(), |f| format!("  ·  LOG {}", f.to_string_lossy()));
            centered(
                &format!("MISSION TIME {}  ·  {} COLLISIONS{log}", clock(t), world.collisions.len()),
                sw / 2.0,
                y + 86.0,
                17.0,
                grey(0.6),
            );
        }
        if let Some(t) = hud.toast {
            centered(t, sw / 2.0, sh * 0.12, 20.0, grey(0.9));
        }
    }

    /// What the drone knows: its self-built map, route, goal, beacon track and home.
    fn draw_minimap(&mut self, world: &World, ap: &Autopilot) {
        let n = ap.map.size() as u16;
        if self.minimap.as_ref().is_none_or(|(_, img)| img.width != n) {
            let img = Image::gen_image_color(n, n, Color::new(0.0, 0.0, 0.0, 0.0));
            let tex = Texture2D::from_image(&img);
            tex.set_filter(FilterMode::Nearest);
            self.minimap = Some((tex, img));
            self.minimap_refresh = 0.0;
        }
        let (tex, img) = self.minimap.as_mut().unwrap();
        if get_time() >= self.minimap_refresh {
            self.minimap_refresh = get_time() + 0.25;
            let data = img.get_image_data_mut();
            for y in 0..n as i32 {
                for x in 0..n as i32 {
                    data[(y * n as i32 + x) as usize] = match ap.map.state((x, y)) {
                        CellState::Unknown => [0, 0, 0, 150],
                        CellState::Free => [34, 34, 38, 230],
                        CellState::Margin => [60, 60, 64, 240],
                        CellState::Wall => [210, 210, 210, 255],
                    };
                }
            }
            tex.update(img);
        }

        let (x0, y0) = (screen_width() - 24.0 - MINIMAP, 24.0);
        draw_texture_ex(
            tex,
            x0,
            y0,
            WHITE,
            DrawTextureParams { dest_size: Some(vec2(MINIMAP, MINIMAP)), ..Default::default() },
        );
        draw_rectangle_lines(x0, y0, MINIMAP, MINIMAP, 1.0, grey(0.4));
        let to_px = |p: Vec2| vec2(x0, y0) + ap.map.to_unit(p) * MINIMAP;

        for w in ap.path.windows(2) {
            let (a, b) = (to_px(w[0]), to_px(w[1]));
            draw_line(a.x, a.y, b.x, b.y, 1.5, BLOOD);
        }
        if let Some(g) = ap.goal {
            let g = to_px(g);
            draw_line(g.x - 4.0, g.y - 4.0, g.x + 4.0, g.y + 4.0, 1.5, BLOOD);
            draw_line(g.x - 4.0, g.y + 4.0, g.x + 4.0, g.y - 4.0, 1.5, BLOOD);
        }
        let h = to_px(vec2(world.home.x, world.home.z));
        draw_rectangle_lines(h.x - 4.0, h.y - 4.0, 8.0, 8.0, 1.5, grey(0.9));
        if let Some(t) = ap.track {
            let p = to_px(vec2(t.pos.x, t.pos.z));
            let fresh = (1.0 - (ap.time - t.seen_at) / 10.0).clamp(0.3, 1.0);
            draw_circle(p.x, p.y, 4.0, Color::new(VIOLET.r, VIOLET.g, VIOLET.b, fresh));
        }
        let s = world.drone_state();
        let d = to_px(vec2(s.pos.x, s.pos.z));
        let nose = d + vec2(-world.yaw().sin(), -world.yaw().cos()) * 9.0;
        draw_line(d.x, d.y, nose.x, nose.y, 1.5, WHITE);
        draw_circle(d.x, d.y, 3.0, WHITE);
        draw_text("N", x0 + MINIMAP / 2.0 - 4.0, y0 - 4.0, 14.0, grey(0.5));
        draw_text(
            format!("DRONE'S MAP · {:.0}% SEEN", ap.map.explored() * 100.0),
            x0,
            y0 + MINIMAP + 16.0,
            15.0,
            grey(0.5),
        );
    }
}

/// A few lit windows per building, placed deterministically on its faces.
fn draw_windows(id: u32, s: Vec3, color: Color) {
    for k in 0..10u32 {
        if hash(id, k) < 0.5 {
            continue;
        }
        let y = (hash(id, k + 50) - 0.5) * s.y * 0.85;
        let along = hash(id, k + 90) - 0.5;
        let (pos, size) = match k % 4 {
            0 => (vec3(s.x / 2.0 + 0.02, y, along * s.z * 0.8), vec3(0.04, 0.7, 0.5)),
            1 => (vec3(-s.x / 2.0 - 0.02, y, along * s.z * 0.8), vec3(0.04, 0.7, 0.5)),
            2 => (vec3(along * s.x * 0.8, y, s.z / 2.0 + 0.02), vec3(0.5, 0.7, 0.04)),
            _ => (vec3(along * s.x * 0.8, y, -s.z / 2.0 - 0.02), vec3(0.5, 0.7, 0.04)),
        };
        draw_cube(pos, size, None, color);
    }
}

/// Draw something in a pose's local space.
fn with_pose(pose: Pose, draw: impl FnOnce()) {
    let m = Mat4::from_rotation_translation(pose.rot, pose.pos);
    unsafe { get_internal_gl() }.quad_gl.push_model_matrix(m);
    draw();
    unsafe { get_internal_gl() }.quad_gl.pop_model_matrix();
}

fn centered(text: &str, cx: f32, y: f32, size: f32, color: Color) {
    let w = measure_text(text, None, size as u16, 1.0).width;
    draw_text(text, cx - w / 2.0, y, size, color);
}

fn wrap_text(text: &str, x: f32, y: f32, size: f32, width: f32, color: Color) {
    let mut line = String::new();
    let mut y = y;
    for word in text.split(' ') {
        let candidate = if line.is_empty() { word.to_string() } else { format!("{line} {word}") };
        if measure_text(&candidate, None, size as u16, 1.0).width > width && !line.is_empty() {
            draw_text(&line, x, y, size, color);
            y += size * 1.3;
            line = word.to_string();
        } else {
            line = candidate;
        }
    }
    draw_text(&line, x, y, size, color);
}

fn grey(v: f32) -> Color {
    Color::new(v, v, v, 1.0)
}

fn mix(a: Color, b: Color, t: f32) -> Color {
    Color::new(a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a)
}

fn hash(i: u32, k: u32) -> f32 {
    let mut h = i.wrapping_mul(374_761_393).wrapping_add(k.wrapping_mul(668_265_263));
    h = (h ^ (h >> 13)).wrapping_mul(1_274_126_177);
    (h ^ (h >> 16)) as f32 / u32::MAX as f32
}
