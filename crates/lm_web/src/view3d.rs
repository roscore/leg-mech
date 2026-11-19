//! 3D leg scene: fixed thigh, swinging shank (knee) or foot plate on a universal joint (ankle),
//! linkage bars and pins, and each actuator as motor can + spinning screw + sliding nut + rod.
//! Orbit camera with side / front presets; drag the shank tip or the toe to set the joint.
use crate::gl::{self, Mesh, Paint, Renderer};
use crate::math::{self, Orbit, OrbitCam, M4, V};
use lm_core::mech::{foot_rot, Kind, Mech};
use lm_core::Inputs;
use std::f32::consts::TAU;

/// Knee axis height above the ground (m).
pub const KNEE_H: f32 = 0.52;
/// Ankle axis height above the ground (m).
pub const ANK_H: f32 = 0.14;
const THIGH: f32 = 0.42;
const SHANK: f32 = 0.40;

/// What a drag in the view asks the app to change.
pub enum Action {
    /// Knee angle (deg).
    Knee(f64),
    /// Ankle pitch / roll increments (deg).
    Ankle(f64, f64),
}

/// Everything the scene needs for one frame.
pub struct State<'a> {
    pub mech: &'a Mech,
    pub q: f64,
    pub roll: f64,
    pub act: &'a Inputs,
    /// Min / max actuator length over the range (m), per actuator.
    pub s_range: [(f64, f64); 2],
    /// Required force exceeds available at this pose.
    pub over: bool,
}

/// Helical thread ridge around a core of radius `r0` up to `r1`, along y from 0 to `len`,
/// triangular profile of half-width `w` (all in metres). Non-indexed (no u16 limit).
pub fn thread(r0: f32, r1: f32, lead: f32, len: f32, w: f32) -> gl::Geo {
    let mut g = gl::Geo::default();
    let seg = 20.0;
    let n = ((len / lead) * seg).ceil().max(1.0) as usize;
    let du = lead / seg;
    let sec = |u: f32| {
        let (s, c) = (TAU * u / lead).sin_cos();
        let p = |r: f32, y: f32| [r * c, y.clamp(0.0, len), r * s];
        [p(r0, u - w), p(r1, u), p(r0, u + w)]
    };
    let mut tri = |a: V, b: V, c: V| {
        let nrm = math::norm(math::cross(math::sub(b, a), math::sub(c, a)));
        for p in [a, b, c] {
            g.pos.extend(p);
            g.nrm.extend(nrm);
        }
    };
    for i in 0..n {
        let (a, b) = (sec(i as f32 * du), sec((i + 1) as f32 * du));
        for k in 0..2 {
            tri(a[k], b[k], b[k + 1]);
            tri(a[k], b[k + 1], a[k + 1]);
        }
    }
    g
}

/// Orthonormal frame with y along `dir` (mesh axis), x / z perpendicular, rotated by `spin`.
fn axis_frame(dir: V, spin: f32) -> (V, V) {
    let helper = if dir[1].abs() < 0.9 { [0.0, 1.0, 0.0] } else { [1.0, 0.0, 0.0] };
    let x0 = math::norm(math::cross(helper, dir));
    let z0 = math::cross(x0, dir);
    let (s, c) = spin.sin_cos();
    (math::add(math::scale(x0, c), math::scale(z0, s)), math::add(math::scale(z0, c), math::scale(x0, -s)))
}

/// Cylinder from `a` along unit `dir`, length `len`, radius `r`, spun by `spin`.
fn along(a: V, dir: V, len: f32, r: f32, spin: f32) -> M4 {
    let (x, z) = axis_frame(dir, spin);
    math::basis(x, dir, z, [2.0 * r, len, 2.0 * r], math::add(a, math::scale(dir, 0.5 * len)))
}

/// Box from `a` to `b` (length axis), thickness `t` along `normal` (made perpendicular), width `w`.
fn bar(a: V, b: V, normal: V, w: f32, t: f32) -> M4 {
    let d = math::sub(b, a);
    let l = math::dot(d, d).sqrt().max(1e-6);
    let x = math::scale(d, 1.0 / l);
    let z = math::norm(math::sub(normal, math::scale(x, math::dot(normal, x))));
    let y = math::cross(z, x);
    math::basis(x, y, z, [l + w * 0.5, w, t], math::scale(math::add(a, b), 0.5))
}

fn knee_w(p: [f64; 2], z: f32) -> V {
    [p[0] as f32, KNEE_H + p[1] as f32, z]
}
fn ank_w(p: [f64; 3]) -> V {
    [p[0] as f32, ANK_H + p[2] as f32, -p[1] as f32]
}

#[derive(Clone, Copy, PartialEq)]
enum Pick {
    None,
    Handle,
}

pub struct Scene {
    pub r: Renderer,
    cyl: Mesh,
    cube: Mesh,
    sphere: Mesh,
    thread: Option<(Mesh, [u64; 3], f32)>,
    pub cam: OrbitCam,
    /// 0 perspective, 1 side, 2 front.
    pub preset: u8,
    handle: V,
    hover: Pick,
    drag: Pick,
    pointers: Vec<(i32, f32, f32)>,
    pinch: f32,
    inv_vp: M4,
    size: (f32, f32),
    ankle: bool,
}

impl Scene {
    pub fn new(r: Renderer) -> Option<Scene> {
        let mut s = Scene {
            cyl: r.mesh(&gl::cylinder(28))?,
            cube: r.mesh(&gl::cube())?,
            sphere: r.mesh(&gl::sphere(16))?,
            r,
            thread: None,
            cam: OrbitCam::new(Orbit { target: [0.0, 0.4, 0.0], yaw: 0.7, pitch: 0.3, dist: 1.4 }),
            preset: 0,
            handle: [0.0; 3],
            hover: Pick::None,
            drag: Pick::None,
            pointers: vec![],
            pinch: 0.0,
            inv_vp: math::IDENTITY,
            size: (1.0, 1.0),
            ankle: false,
        };
        s.r.cell = 0.05;
        s.r.fog = [2.5, 10.0];
        Some(s)
    }

    fn preset_orbit(&self, ankle: bool) -> Orbit {
        let (target, dist) = if ankle { ([0.0, ANK_H + 0.12, 0.0], 0.85) } else { ([0.0, KNEE_H - 0.02, 0.0], 1.45) };
        match self.preset {
            1 => Orbit { target, yaw: 0.0, pitch: 0.02, dist },
            2 => Orbit { target, yaw: std::f32::consts::FRAC_PI_2, pitch: 0.02, dist },
            _ => Orbit { target, yaw: 0.75, pitch: 0.3, dist },
        }
    }

    pub fn reset_view(&mut self, mech: &Mech) {
        let o = self.preset_orbit(matches!(mech.kind, Kind::Ankle(_)));
        self.cam.set_goal(o);
    }

    fn screw_mesh(&mut self, inp: &Inputs, len: f32) -> f32 {
        let d = inp.d_mm as f32 / 1000.0;
        let lead_vis = (inp.lead_mm as f32 / 1000.0).max(len / 80.0);
        let key = [inp.d_mm.to_bits(), (lead_vis as f64).to_bits(), (len as f64).to_bits()];
        if self.thread.as_ref().map(|t| t.1) != Some(key) {
            if let Some((m, _, _)) = self.thread.take() {
                self.r.free(m);
            }
            let dr = lm_core::root_diameter(inp.d_mm, inp.lead_mm) as f32 / 1000.0;
            let g = thread(0.5 * dr, 0.5 * d, lead_vis, len, (0.35 * lead_vis).min(0.25 * d));
            self.thread = self.r.mesh(&g).map(|m| (m, key, lead_vis));
        }
        lead_vis
    }

    pub fn frame(&mut self, w: i32, h: i32, dt: f32, st: &State) {
        self.ankle = matches!(st.mech.kind, Kind::Ankle(_));
        self.cam.step(dt);
        let eye = self.cam.cur.eye();
        let view = math::look_at(eye, self.cam.cur.target, [0.0, 1.0, 0.0]);
        let proj = math::perspective(0.8, w as f32 / h.max(1) as f32, 0.01, 60.0);
        self.inv_vp = math::invert(&math::mul(&proj, &view)).unwrap_or(math::IDENTITY);
        self.size = (w as f32, h as f32);
        self.r.begin(&view, &proj, eye, [0, 0, w, h]);

        let dark = self.r.theme.shadow > 0.4;
        let body = if dark { [0.62, 0.65, 0.70] } else { [0.78, 0.80, 0.84] };
        let accent = [0.22, 0.45, 0.82];
        let pin = [0.25, 0.27, 0.30];
        let steel = [0.72, 0.74, 0.78];
        let can = [0.16, 0.17, 0.19];
        let red = [0.92, 0.30, 0.25];
        let nut_rgb = if st.over { red } else {
            match st.act.kind {
                lm_core::Kind::TrapBronze => [0.80, 0.55, 0.25],
                lm_core::Kind::TrapPom => [0.92, 0.92, 0.88],
                _ => [0.30, 0.55, 0.40],
            }
        };
        let rgba = |c: [f32; 3]| [c[0], c[1], c[2], 1.0];
        let lit = |c: [f32; 3]| Paint::lit(rgba(c));
        let glow = if self.hover == Pick::Handle || self.drag == Pick::Handle { 0.3 } else { 0.0 };
        // Collected as (mesh id, model, paint) so shadows and colours share the list.
        let mut parts: Vec<(u8, M4, Paint)> = vec![];
        const CYL: u8 = 0;
        const CUBE: u8 = 1;
        const SPH: u8 = 2;
        const THR: u8 = 3;
        let z = [0.0, 0.0, 1.0];
        // Actuators to draw: (base pivot, rod end, actuator index).
        let mut acts: Vec<(V, V, usize)> = vec![];

        match st.mech.kind {
            Kind::SliderCrank(_) | Kind::FourBar(_) => {
                let q = st.q;
                let (sq, cq) = (q.sin() as f32, q.cos() as f32);
                let sd = [-sq, -cq, 0.0];
                let knee = [0.0, KNEE_H, 0.0];
                let tip = math::add(knee, math::scale(sd, SHANK));
                // Fixture: stand post and arm holding the hip block (thigh is fixed).
                let hip = [0.0, KNEE_H + THIGH, 0.0];
                parts.push((CUBE, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.04, KNEE_H + THIGH + 0.03, 0.04], [-0.22, 0.5 * (KNEE_H + THIGH + 0.03), 0.0]), lit([0.40, 0.42, 0.46])));
                parts.push((CUBE, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.24, 0.035, 0.05], [-0.11, KNEE_H + THIGH, 0.0]), lit([0.40, 0.42, 0.46])));
                parts.push((CUBE, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.11, 0.05, 0.11], hip), lit(body)));
                parts.push((CUBE, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.40, 0.012, 0.30], [-0.12, 0.006, 0.0]), lit([0.40, 0.42, 0.46])));
                // Thigh and shank: twin side plates around the actuator plane z = 0.
                for zs in [0.045f32, -0.045] {
                    parts.push((CUBE, bar([0.0, KNEE_H, zs], [0.0, KNEE_H + THIGH, zs], z, 0.075, 0.012), lit(body)));
                    let mut sp = lit(accent);
                    sp.glow = glow;
                    let k = [0.0, KNEE_H, zs * 0.8];
                    parts.push((CUBE, bar(k, math::add(k, math::scale(sd, SHANK)), z, 0.06, 0.012), sp));
                }
                parts.push((CYL, along([0.0, KNEE_H, -0.06], z, 0.12, 0.02, 0.0), lit(pin)));
                // Foot under the shank tip (ankle neutral) — the drag handle.
                let mut fp = lit(accent);
                fp.glow = glow;
                let foot_fwd = [cq, -sq, 0.0];
                parts.push((CUBE, math::basis(foot_fwd, [-sd[0], -sd[1], 0.0], z, [0.20, 0.035, 0.09], math::add(tip, math::scale(foot_fwd, 0.04))), fp));
                parts.push((SPH, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.05; 3], tip), fp));
                self.handle = tip;
                let (a, b) = match st.mech.kind {
                    Kind::SliderCrank(m) => {
                        let b = m.rod_end(q);
                        parts.push((CUBE, bar(knee, knee_w(b, 0.0), z, 0.03, 0.014), lit(accent)));
                        (knee_w(m.base(), 0.0), knee_w(b, 0.0))
                    }
                    Kind::FourBar(m) => match m.solve(q) {
                        Some(p) => {
                            let c = knee_w([m.cx, m.cy], 0.0);
                            let (k, pp, b) = (knee_w(p.k, 0.0), knee_w(p.p, 0.0), knee_w(p.b, 0.0));
                            parts.push((CUBE, bar(knee, b, z, 0.028, 0.014), lit(accent)));
                            parts.push((CUBE, bar(c, k, z, 0.026, 0.016), lit([0.85, 0.55, 0.2])));
                            parts.push((CUBE, bar(c, pp, z, 0.026, 0.016), lit([0.85, 0.55, 0.2])));
                            parts.push((CUBE, bar(k, pp, z, 0.02, 0.012), lit([0.85, 0.55, 0.2])));
                            parts.push((CUBE, bar(k, b, z, 0.018, 0.01), lit([0.55, 0.58, 0.62])));
                            for pt in [c, k, b] {
                                parts.push((CYL, along(math::add(pt, [0.0, 0.0, -0.05]), z, 0.1, 0.008, 0.0), lit(pin)));
                            }
                            (knee_w([m.ax, m.ay], 0.0), pp)
                        }
                        None => (knee_w([m.ax, m.ay], 0.0), knee_w([m.cx, m.cy], 0.0)),
                    },
                    Kind::Ankle(_) => unreachable!(),
                };
                for pt in [a, b] {
                    parts.push((CYL, along(math::add(pt, [0.0, 0.0, -0.05]), z, 0.1, 0.007, 0.0), lit(pin)));
                }
                acts.push((a, b, 0));
            }
            Kind::Ankle(an) => {
                let (p, r) = (st.q, st.roll);
                let ank = [0.0, ANK_H, 0.0];
                // Shank post, upper bracket, fixture.
                parts.push((CUBE, bar(ank, [0.0, ANK_H + SHANK, 0.0], [1.0, 0.0, 0.0], 0.05, 0.05), lit(body)));
                let ub = ank_w([-an.c, 0.0, an.h]);
                parts.push((CUBE, math::basis([1., 0., 0.], [0., 1., 0.], z, [an.c as f32 + 0.03, 0.025, an.w as f32 + 0.03], [-0.5 * an.c as f32, ub[1], 0.0]), lit(body)));
                parts.push((CUBE, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.05, ANK_H + SHANK + 0.03, 0.05], [0.2, 0.5 * (ANK_H + SHANK + 0.03), 0.0]), lit([0.40, 0.42, 0.46])));
                parts.push((CUBE, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.2, 0.03, 0.05], [0.1, ANK_H + SHANK, 0.0]), lit([0.40, 0.42, 0.46])));
                // Universal joint cross.
                parts.push((CYL, along([0.0, ANK_H, -0.04], z, 0.08, 0.012, 0.0), lit(pin)));
                let rx = ank_w(foot_rot(p, 0.0, [1.0, 0.0, 0.0]));
                let rx = math::sub(rx, ank);
                parts.push((CYL, along(math::sub(ank, math::scale(rx, 0.035)), rx, 0.07, 0.011, 0.0), lit(pin)));
                // Foot plate in the foot frame.
                let fr = |v: [f64; 3]| math::sub(ank_w(foot_rot(p, r, v)), ank);
                let (fx, fy, fz) = (fr([1.0, 0.0, 0.0]), fr([0.0, 1.0, 0.0]), fr([0.0, 0.0, 1.0]));
                let mut fp = lit(accent);
                fp.glow = glow;
                let centre = ank_w(foot_rot(p, r, [0.04, 0.0, -0.06]));
                parts.push((CUBE, math::basis(fx, fz, fy, [0.25, 0.022, 0.10], centre), fp));
                let toe = ank_w(foot_rot(p, r, [0.165, 0.0, -0.06]));
                parts.push((SPH, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.045; 3], toe), fp));
                self.handle = toe;
                // Heel brackets from the lower pivots down to the plate; ankle post.
                parts.push((CUBE, bar(ank, ank_w(foot_rot(p, r, [0.0, 0.0, -0.06])), fx, 0.03, 0.03), lit(body)));
                for i in 0..2 {
                    let lo = ank_w(an.lower(i, p, r));
                    let pl = ank_w(foot_rot(p, r, sub_z(an.foot_local(i), 0.06)));
                    parts.push((CUBE, bar(lo, pl, fx, 0.022, 0.014), lit(body)));
                    let up = ank_w(an.upper(i));
                    parts.push((SPH, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.018; 3], up), lit(pin)));
                    parts.push((SPH, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.018; 3], lo), lit(pin)));
                    acts.push((up, lo, i));
                }
            }
        }

        // Actuators: motor can at the base pivot, screw, nut at s − rod length, rod to the rod end.
        let d = (st.act.d_mm as f32 / 1000.0).clamp(0.006, 0.04);
        let nut = (lm_core::couple::nut_length_m(st.act.d_mm) as f32).min(0.08);
        let stroke = acts.iter().map(|a| (st.s_range[a.2].1 - st.s_range[a.2].0) as f32).fold(0.0, f32::max).max(0.005);
        let screw_len = stroke + nut;
        let lead_vis = self.screw_mesh(st.act, screw_len);
        let mut thread_parts: Vec<M4> = vec![];
        for &(a, b, i) in &acts {
            let dv = math::sub(b, a);
            let s = math::dot(dv, dv).sqrt().max(1e-4);
            let dir = math::scale(dv, 1.0 / s);
            let (s_min, s_max) = (st.s_range[i].0 as f32, st.s_range[i].1 as f32);
            let mr = (0.5 * d + 0.012).max(0.018);
            let m_len = (0.25 * s_min).clamp(0.02, 0.06);
            let rod_len = (s_max - m_len - screw_len + 0.5 * nut).max(0.01);
            let xn = (s - rod_len).clamp(m_len + 0.5 * nut, m_len + screw_len - 0.5 * nut);
            let spin = TAU * (s - s_min) / lead_vis;
            parts.push((CYL, along(a, dir, m_len, mr, 0.0), lit(can)));
            parts.push((CYL, along(math::add(a, math::scale(dir, m_len - 0.004)), dir, 0.004, mr + 0.003, 0.0), lit(if st.over { red } else { [0.3, 0.3, 0.32] })));
            let s0 = math::add(a, math::scale(dir, m_len));
            parts.push((CYL, along(s0, dir, screw_len, 0.5 * lm_core::root_diameter(st.act.d_mm, st.act.lead_mm) as f32 / 1000.0, spin), lit(if st.over { red } else { steel })));
            let (x, zz) = axis_frame(dir, spin);
            thread_parts.push(math::basis(x, dir, zz, [1.0; 3], s0));
            parts.push((CYL, along(math::add(a, math::scale(dir, xn - 0.5 * nut)), dir, nut, 0.5 * d + 0.007, 0.0), lit(nut_rgb)));
            parts.push((CYL, along(math::add(a, math::scale(dir, xn)), dir, s - xn, 0.25 * d + 0.002, 0.0), lit(steel)));
            parts.push((SPH, math::basis([1., 0., 0.], [0., 1., 0.], z, [0.016; 3], b), lit(pin)));
        }
        let thread_rgb = if st.over { red } else { steel };
        for m in thread_parts {
            parts.push((THR, m, lit(thread_rgb)));
        }

        let mesh = |id: u8| -> Option<&Mesh> {
            match id {
                CYL => Some(&self.cyl),
                CUBE => Some(&self.cube),
                SPH => Some(&self.sphere),
                _ => self.thread.as_ref().map(|t| &t.0),
            }
        };
        for (id, model, _) in &parts {
            if let Some(m) = mesh(*id) {
                self.r.shadow(m, model);
            }
        }
        for (id, model, p) in &parts {
            if let Some(m) = mesh(*id) {
                self.r.draw(m, model, *p);
            }
        }
    }

    fn ray(&self, x: f32, y: f32) -> (V, V) {
        math::ndc_ray(&self.inv_vp, 2.0 * x / self.size.0 - 1.0, 1.0 - 2.0 * y / self.size.1)
    }

    fn pick(&self, x: f32, y: f32) -> Pick {
        let (o, d) = self.ray(x, y);
        if math::ray_sphere(o, d, self.handle, 0.06).is_some() { Pick::Handle } else { Pick::None }
    }

    pub fn down(&mut self, id: i32, x: f32, y: f32) {
        self.pointers.retain(|p| p.0 != id);
        self.pointers.push((id, x, y));
        if self.pointers.len() == 1 {
            self.drag = self.pick(x, y);
            self.cam.held = self.drag == Pick::None;
        } else {
            self.drag = Pick::None;
            self.pinch = self.spread();
        }
    }

    fn spread(&self) -> f32 {
        match self.pointers.as_slice() {
            [a, b, ..] => ((a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt(),
            _ => 0.0,
        }
    }

    fn pan(&mut self, dx: f32, dy: f32) {
        let c = self.cam.cur;
        let f = math::norm(math::sub(c.target, c.eye()));
        let right = math::norm(math::cross(f, [0.0, 1.0, 0.0]));
        let up = math::cross(right, f);
        let k = c.dist * 0.0015;
        self.cam.pan(math::add(math::scale(right, -dx * k), math::scale(up, dy * k)));
    }

    /// Pointer move; `buttons` is the MouseEvent buttons mask. Returns a joint change, if any.
    pub fn moved(&mut self, id: i32, x: f32, y: f32, buttons: u16) -> (Option<Action>, &'static str) {
        let Some(i) = self.pointers.iter().position(|p| p.0 == id) else {
            self.hover = self.pick(x, y);
            return (None, if self.hover == Pick::None { "default" } else { "grab" });
        };
        let (dx, dy) = (x - self.pointers[i].1, y - self.pointers[i].2);
        self.pointers[i] = (id, x, y);
        if self.pointers.len() >= 2 {
            let s = self.spread();
            if self.pinch > 0.0 && s > 0.0 {
                self.cam.zoom(self.pinch / s);
            }
            self.pinch = s;
            self.pan(0.5 * dx, 0.5 * dy);
            return (None, "default");
        }
        let act = match self.drag {
            Pick::Handle if self.ankle => Some(Action::Ankle(dy as f64 * 0.25, -dx as f64 * 0.25)),
            Pick::Handle => {
                // Point under the pointer on the sagittal plane z = 0, angle of knee → point.
                let (o, d) = self.ray(x, y);
                math::ray_plane(o, d, [0.0, KNEE_H, 0.0], [0.0, 0.0, 1.0]).map(|p| {
                    let (px, py) = (p[0], p[1] - KNEE_H);
                    Action::Knee((-px).atan2(-py).to_degrees() as f64)
                })
            }
            Pick::None => {
                if buttons & 2 != 0 {
                    self.pan(dx, dy);
                } else {
                    self.cam.rotate(-dx * 0.008, dy * 0.008);
                }
                None
            }
        };
        (act, if self.drag == Pick::None { "default" } else { "grabbing" })
    }

    pub fn up(&mut self, id: i32) {
        self.pointers.retain(|p| p.0 != id);
        if self.pointers.is_empty() {
            self.drag = Pick::None;
            self.cam.held = false;
        }
    }
}

fn sub_z(v: [f64; 3], dz: f64) -> [f64; 3] {
    [v[0], v[1], v[2] - dz]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thread_mesh_size_and_bounds() {
        let g = thread(0.006, 0.008, 0.005, 0.1, 0.0015);
        // 20 turns × 20 segments × 2 faces × 2 triangles × 3 vertices.
        assert_eq!(g.pos.len() / 3, 20 * 20 * 12);
        assert!(g.pos.chunks(3).all(|p| (0.0..=0.1).contains(&p[1]) && (p[0] * p[0] + p[2] * p[2]).sqrt() <= 0.00801));
    }

    #[test]
    fn axis_frame_is_orthonormal() {
        for dir in [[0.0, 1.0, 0.0], math::norm([0.3, -0.2, 0.9])] {
            let (x, z) = axis_frame(dir, 0.7);
            for (a, b) in [(x, dir), (z, dir), (x, z)] {
                assert!(math::dot(a, b).abs() < 1e-5);
            }
            assert!((math::dot(x, x) - 1.0).abs() < 1e-5 && (math::dot(z, z) - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn world_mappings_keep_knee_and_ankle_axes() {
        assert_eq!(knee_w([0.0, 0.0], 0.0), [0.0, KNEE_H, 0.0]);
        // Ankle frame (x fwd, y left, z up) → world (x, y up, z = −left), right-handed.
        assert_eq!(ank_w([0.0, 1.0, 0.0]), [0.0, ANK_H, -1.0]);
        assert_eq!(ank_w([0.0, 0.0, 1.0]), [0.0, ANK_H + 1.0, 0.0]);
    }
}
