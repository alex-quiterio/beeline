//! Noir viewer: black-and-white night city, fog, rain driven by the wind, film grain and vignette.
//! Red is the only colour, and it marks what matters: the drone and its current target.
//! Pure presentation; it only reads the world and the autopilot.

use beeline::flight::{wrap_angle, FlightController};
use beeline::nav::{Autopilot, Status};
use beeline::sim::city::{Kind, Shape};
use beeline::sim::{Pose, World};
use macroquad::prelude::*;

const NIGHT: Color = Color::new(0.03, 0.03, 0.04, 1.0);
const BLOOD: Color = Color::new(0.82, 0.07, 0.07, 1.0);
const LAMP: Color = Color::new(0.95, 0.85, 0.6, 1.0); // lit windows
const FOG_DIST: f32 = 120.0;

pub struct Renderer {
    cam: Vec3,
    cam_yaw: f32,
    wind_travel: Vec3,
}

impl Renderer {
    pub fn new() -> Self {
        Self { cam: vec3(0.0, 3.5, 6.0), cam_yaw: 0.0, wind_travel: Vec3::ZERO }
    }

    pub fn draw(&mut self, world: &World, ap: &Autopilot, fc: &FlightController, auto: bool, alpha: f32, dt: f32) {
        let drone = world.drone_pose(alpha);

        // Chase camera, smoothed independently of frame rate
        self.cam_yaw += wrap_angle(world.yaw() - self.cam_yaw) * (1.0 - (-4.0 * dt).exp());
        let wanted = drone.pos + Quat::from_rotation_y(self.cam_yaw) * vec3(0.0, 2.5, 6.0);
        self.cam = self.cam.lerp(wanted, 1.0 - (-10.0 * dt).exp());
        self.wind_travel += world.wind() * dt;
        let cam = self.cam;
        let fog = |c: Color, p: Vec3| mix(c, NIGHT, (cam.distance(p) / FOG_DIST).clamp(0.0, 1.0).powf(0.7));

        clear_background(NIGHT);
        set_camera(&Camera3D { position: cam, target: drone.pos, up: Vec3::Y, ..Default::default() });

        draw_plane(vec3(0.0, -0.01, 0.0), vec2(300.0, 300.0), None, Color::new(0.06, 0.06, 0.07, 1.0));
        draw_grid(40, 16.0, Color::new(0.16, 0.16, 0.17, 1.0), Color::new(0.1, 0.1, 0.11, 1.0));

        // City
        for (i, o) in world.city.obstacles.iter().enumerate() {
            let pose = world.obstacle_pose(i, alpha);
            let (body, edge) = match o.kind {
                Kind::Building { shade } => (grey(0.05 + shade * 0.22), grey(0.3)),
                Kind::Gate => (grey(0.18), grey(0.45)),
                Kind::Lift => (grey(0.3), grey(0.6)),
                Kind::Fan => (grey(0.1), grey(0.35)),
                Kind::Patrol => (grey(0.85), grey(0.85)), // pale "searchlight" orbs
            };
            let (body, edge) = (fog(body, pose.pos), fog(edge, pose.pos));
            let lamp = fog(LAMP, pose.pos);
            with_pose(pose, || match o.shape {
                Shape::Box(s) => {
                    draw_cube(Vec3::ZERO, s, None, body);
                    draw_cube_wires(Vec3::ZERO, s, edge);
                    if matches!(o.kind, Kind::Building { .. }) {
                        draw_windows(i as u32, s, lamp);
                    }
                }
                Shape::Ball(r) => draw_sphere(Vec3::ZERO, r, None, body),
            });
        }

        // Mission: waypoints, planned route, sensors
        let wps = &ap.waypoints;
        for (i, wp) in wps.iter().enumerate() {
            let next = wps[(i + 1) % wps.len()];
            draw_line_3d(vec3(wp.x, 0.05, wp.z), vec3(next.x, 0.05, next.z), Color::new(0.5, 0.5, 0.5, 0.25));
            draw_sphere(*wp, 0.35, None, if i == ap.current { BLOOD } else { fog(grey(0.55), *wp) });
        }
        if auto {
            for w in ap.path.windows(2) {
                draw_line_3d(vec3(w[0].x, 0.1, w[0].y), vec3(w[1].x, 0.1, w[1].y), Color::new(0.82, 0.07, 0.07, 0.6));
            }
            for r in &ap.rays {
                let c = if r.close { Color::new(0.82, 0.07, 0.07, 0.7) } else { Color::new(1.0, 1.0, 1.0, 0.08) };
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
        self.draw_hud(world, ap, fc, auto);
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

    fn draw_hud(&self, world: &World, ap: &Autopilot, fc: &FlightController, auto: bool) {
        set_default_camera();
        let (sw, sh) = (screen_width(), screen_height());

        // Vignette and film grain
        for i in 0..14 {
            let inset = i as f32 * 14.0;
            draw_rectangle_lines(inset, inset, sw - 2.0 * inset, sh - 2.0 * inset, 30.0,
                Color::new(0.0, 0.0, 0.0, 0.32 * (1.0 - i as f32 / 14.0)));
        }
        for _ in 0..350 {
            let a = rand::gen_range(0.02, 0.09);
            draw_rectangle(rand::gen_range(0.0, sw), rand::gen_range(0.0, sh), 1.5, 1.5, Color::new(1.0, 1.0, 1.0, a));
        }

        let s = world.drone_state();
        let dim = grey(0.55);
        draw_text("B E E L I N E", 24.0, 34.0, 26.0, grey(0.85));
        draw_text(format!("ALT {:>5.1} m    SPD {:>4.1} m/s    WIND {:>3.1}    HITS {}",
            s.pos.y, s.vel.length(), world.wind().length(), world.collisions.len()), 24.0, 60.0, 20.0, dim);
        if auto {
            let busy = ap.status != Status::Following;
            draw_text(format!("AUTOPILOT   WP {}/{}   LAP {}   {}", ap.current + 1, ap.waypoints.len(), ap.laps,
                ap.status.label()), 24.0, 84.0, 20.0, if busy { BLOOD } else { grey(0.85) });
        } else {
            draw_text("MANUAL", 24.0, 84.0, 20.0, grey(0.85));
        }
        draw_text("ARROWS move   W/S climb   A/D turn   T autopilot   R reset", 24.0, sh - 24.0, 18.0, grey(0.4));

        // Bee "eyes": optic flow on each side (centring = keeping them balanced)
        if auto {
            let (cx, y) = (sw / 2.0, sh - 56.0);
            let (l, r) = ((ap.flow.left * 40.0).min(160.0), (ap.flow.right * 40.0).min(160.0));
            draw_rectangle(cx - 6.0 - l, y, l, 6.0, grey(0.7));
            draw_rectangle(cx + 6.0, y, r, 6.0, grey(0.7));
            draw_line(cx, y - 6.0, cx, y + 12.0, 1.0, BLOOD);
            draw_text("OPTIC FLOW", cx - 38.0, y - 10.0, 16.0, grey(0.45));
        }

        // Wind compass, relative to the view: true wind (white) vs the controller's estimate (red)
        let c = vec2(sw - 60.0, 60.0);
        let to_screen = |w: Vec3| {
            let r = Quat::from_rotation_y(-self.cam_yaw) * w;
            c + vec2(r.x, r.z) * 8.0
        };
        draw_circle_lines(c.x, c.y, 36.0, 1.0, grey(0.5));
        let (t, e) = (to_screen(world.wind()), to_screen(fc.wind_estimate()));
        draw_line(c.x, c.y, t.x, t.y, 2.0, grey(0.85));
        draw_line(c.x, c.y, e.x, e.y, 2.0, BLOOD);
        draw_text("WIND", c.x - 16.0, c.y + 54.0, 16.0, grey(0.45));
    }
}

/// A few lit windows per building, placed deterministically on its faces.
fn draw_windows(id: u32, s: Vec3, color: Color) {
    for k in 0..10u32 {
        if hash(id, k) < 0.5 { continue; }
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
