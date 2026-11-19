//! Planar knee linkages and a two-actuator parallel ankle (pure Rust, host-testable).
//!
//! Textbook kinematics only. SI internally (m, rad); degrees appear only in field names `*_deg`.
//!
//! Knee frame (sagittal plane): origin on the knee axis, x forward, y up. The thigh is fixed
//! and points up (+y). At q = 0 the shank hangs straight down; flexion q > 0 turns the shank
//! backward, i.e. shank-fixed points rotate by −q (clockwise seen from the robot's left).
//!
//! Ankle frame: origin on the ankle axis, x forward, y to the left, z up. Foot orientation is
//! R = Ry(pitch) · Rx(roll): the pitch axis (y) is fixed in the shank and carries the roll axis.

use std::f64::consts::PI;

pub type V2 = [f64; 2];
pub type V3 = [f64; 3];

fn rot2(p: V2, a: f64) -> V2 {
    let (s, c) = a.sin_cos();
    [c * p[0] - s * p[1], s * p[0] + c * p[1]]
}
fn sub2(a: V2, b: V2) -> V2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn dot2(a: V2, b: V2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn len2(a: V2) -> f64 {
    dot2(a, a).sqrt()
}
fn cross2(a: V2, b: V2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
/// Derivative of rot2(p, a) w.r.t. a.
fn drot2(p: V2, a: f64) -> V2 {
    let (s, c) = a.sin_cos();
    [-s * p[0] - c * p[1], c * p[0] - s * p[1]]
}

pub fn sub3(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn dot3(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross3(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn len3(a: V3) -> f64 {
    dot3(a, a).sqrt()
}

/// Mechanism quality grade of one pose.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Flag {
    Ok,
    Warn,
    Fail,
}

/// Transmission angle below which a pose is a warning / failure (deg).
pub const MU_WARN_DEG: f64 = 20.0;
pub const MU_FAIL_DEG: f64 = 10.0;
/// Moment arm (as a fraction of the lever radius) below which a single-rod pose is a dead centre.
pub const H_FAIL_FRAC: f64 = 0.05;
/// |det J| normalised by (c · w/2) below which an ankle pose counts as singular.
pub const DET_SINGULAR: f64 = 0.05;

fn grade_mu(mu_deg: f64) -> Flag {
    if !(mu_deg >= MU_FAIL_DEG) {
        Flag::Fail
    } else if mu_deg < MU_WARN_DEG {
        Flag::Warn
    } else {
        Flag::Ok
    }
}

// ───────────────────────────── Knee: slider-crank ─────────────────────────────

/// Actuator base pivot A on the thigh, rod end B on a shank lever of radius `r`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SliderCrank {
    /// Base pivot (m) from the knee axis.
    pub ax: f64,
    pub ay: f64,
    /// Lever radius (m).
    pub r: f64,
    /// Lever angle at q = 0 (deg), measured CCW from +x.
    pub q0_deg: f64,
}

impl SliderCrank {
    /// Rod end B(q) (m).
    pub fn rod_end(&self, q: f64) -> V2 {
        rot2([self.r, 0.0], self.q0_deg.to_radians() - q)
    }
    pub fn base(&self) -> V2 {
        [self.ax, self.ay]
    }
    /// Actuator length s(q) (m).
    pub fn length(&self, q: f64) -> f64 {
        len2(sub2(self.rod_end(q), self.base()))
    }
    /// J(q) = ds/dq (m/rad), analytic: (B − A)·dB/dq / s with dB/dq = −∂rot.
    pub fn jac(&self, q: f64) -> f64 {
        let d = sub2(self.rod_end(q), self.base());
        let db = drot2([self.r, 0.0], self.q0_deg.to_radians() - q);
        -dot2(d, db) / len2(d).max(1e-12)
    }
    /// Angle between rod and lever (deg, 0…90): 90 means the rod pushes square on the lever.
    pub fn rod_lever_angle_deg(&self, q: f64) -> f64 {
        let b = self.rod_end(q);
        let d = sub2(b, self.base());
        (cross2(b, d).abs() / (len2(b) * len2(d)).max(1e-12)).clamp(0.0, 1.0).asin().to_degrees()
    }
}

// ───────────────────────────── Knee: four-bar ─────────────────────────────

/// Actuator → bell-crank on the thigh → coupler → shank lever.
///
/// Crank pivot C on the thigh. Crank arm to the coupler joint K has length `rc`; the actuator
/// rod attaches to the same rigid crank at P, radius `rp`, rotated `delta` from the K arm.
/// The coupler (length `lc`) joins K to the shank lever point B (radius `rb`, angle `q0` at q = 0).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FourBar {
    /// Actuator base pivot on the thigh (m).
    pub ax: f64,
    pub ay: f64,
    /// Crank pivot on the thigh (m).
    pub cx: f64,
    pub cy: f64,
    /// Crank arm to coupler joint (m).
    pub rc: f64,
    /// Crank arm to actuator rod end (m).
    pub rp: f64,
    /// Angle from the K arm to the P arm (deg).
    pub delta_deg: f64,
    /// Coupler length (m).
    pub lc: f64,
    /// Shank lever radius (m) and angle at q = 0 (deg).
    pub rb: f64,
    pub q0_deg: f64,
    /// Assembly branch: +1 or −1 (which side of line C–B the joint K sits on).
    pub branch: f64,
}

/// One solved four-bar pose.
#[derive(Clone, Copy, Debug)]
pub struct FourBarPose {
    pub b: V2,
    pub k: V2,
    pub p: V2,
    /// Crank angle of the K arm (rad).
    pub psi: f64,
    pub s: f64,
    /// ds/dq (m/rad).
    pub j: f64,
    /// Output transmission angle (coupler vs shank lever) and input angle (coupler vs crank), deg 0…90.
    pub mu_out_deg: f64,
    pub mu_in_deg: f64,
    /// Height of triangle C–K–B relative to min(rc, lc): → 0 where the two assembly modes merge.
    pub branch_margin: f64,
}

impl FourBar {
    pub fn lever(&self, q: f64) -> V2 {
        rot2([self.rb, 0.0], self.q0_deg.to_radians() - q)
    }
    /// Closed-form circle intersection |K − C| = rc, |K − B| = lc. `None` if it cannot assemble.
    pub fn solve(&self, q: f64) -> Option<FourBarPose> {
        let c = [self.cx, self.cy];
        let b = self.lever(q);
        let cb = sub2(b, c);
        let d = len2(cb);
        if d < 1e-12 {
            return None;
        }
        let a = (self.rc * self.rc - self.lc * self.lc + d * d) / (2.0 * d);
        let h2 = self.rc * self.rc - a * a;
        if h2 < 0.0 {
            return None;
        }
        let h = h2.sqrt();
        let u = [cb[0] / d, cb[1] / d];
        let nrm = [-u[1], u[0]];
        let sg = if self.branch < 0.0 { -1.0 } else { 1.0 };
        let k = [c[0] + a * u[0] + sg * h * nrm[0], c[1] + a * u[1] + sg * h * nrm[1]];
        let kc = sub2(k, c);
        let psi = kc[1].atan2(kc[0]);
        let delta = self.delta_deg.to_radians();
        let p = [c[0] + self.rp * (psi + delta).cos(), c[1] + self.rp * (psi + delta).sin()];
        let pa = sub2(p, [self.ax, self.ay]);
        let s = len2(pa);
        // dψ/dq from differentiating |K(ψ) − B(q)|² = lc².
        let kb = sub2(k, b);
        let dk = [-kc[1], kc[0]];
        let db = drot2([self.rb, 0.0], self.q0_deg.to_radians() - q);
        let db = [-db[0], -db[1]];
        let den = dot2(kb, dk);
        let dpsi = if den.abs() > 1e-15 { dot2(kb, db) / den } else { f64::INFINITY.copysign(den) };
        let dp = [-self.rp * (psi + delta).sin(), self.rp * (psi + delta).cos()];
        let ds_dpsi = dot2(pa, dp) / s.max(1e-12);
        let j = ds_dpsi * dpsi;
        let acute = |x: V2, y: V2| (cross2(x, y).abs() / (len2(x) * len2(y)).max(1e-15)).clamp(0.0, 1.0).asin().to_degrees();
        Some(FourBarPose {
            b,
            k,
            p,
            psi,
            s,
            j: if j.is_finite() { j } else { 0.0 },
            mu_out_deg: acute(kb, b),
            mu_in_deg: acute(kb, kc),
            branch_margin: h / self.rc.min(self.lc).max(1e-12),
        })
    }
}

// ───────────────────────────── Ankle: two parallel rods ─────────────────────────────

/// Two actuators from the shank to the foot plate at ±w/2 lateral.
/// Upper pivots U_i = (−c, ±w/2, H) on the shank, lower pivots f_i = (−c, ±w/2, 0) on the foot
/// (foot lever c behind the ankle axis). Actuator 0 is on the +y (left) side.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ankle {
    /// Lateral spacing of the two rods (m).
    pub w: f64,
    /// Upper attachment height above the ankle axis (m).
    pub h: f64,
    /// Foot lever: distance of the foot attachments behind the ankle axis (m).
    pub c: f64,
}

/// Foot rotation R = Ry(p) · Rx(r) applied to `v`.
pub fn foot_rot(p: f64, r: f64, v: V3) -> V3 {
    let (sr, cr) = r.sin_cos();
    let x = [v[0], cr * v[1] - sr * v[2], sr * v[1] + cr * v[2]];
    let (sp, cp) = p.sin_cos();
    [cp * x[0] + sp * x[2], x[1], -sp * x[0] + cp * x[2]]
}

impl Ankle {
    pub fn upper(&self, i: usize) -> V3 {
        [-self.c, if i == 0 { 0.5 } else { -0.5 } * self.w, self.h]
    }
    pub fn foot_local(&self, i: usize) -> V3 {
        [-self.c, if i == 0 { 0.5 } else { -0.5 } * self.w, 0.0]
    }
    pub fn lower(&self, i: usize, p: f64, r: f64) -> V3 {
        foot_rot(p, r, self.foot_local(i))
    }
    /// Actuator lengths (s1, s2) for pitch p, roll r (rad), by vector loop.
    pub fn lengths(&self, p: f64, r: f64) -> [f64; 2] {
        [0, 1].map(|i| len3(sub3(self.lower(i, p, r), self.upper(i))))
    }
    /// J = ∂(s1, s2)/∂(p, r) (m/rad), analytic: J_ij = n_i · ∂(R f_i)/∂q_j.
    pub fn jac(&self, p: f64, r: f64) -> [[f64; 2]; 2] {
        let (sp, cp) = p.sin_cos();
        let (sr, cr) = r.sin_cos();
        [0, 1].map(|i| {
            let f = self.foot_local(i);
            let d = sub3(self.lower(i, p, r), self.upper(i));
            let n = d.map(|x| x / len3(d).max(1e-12));
            // x = Rx(r) f and its derivative.
            let x = [f[0], cr * f[1] - sr * f[2], sr * f[1] + cr * f[2]];
            let dx = [0.0, -sr * f[1] - cr * f[2], cr * f[1] - sr * f[2]];
            let dp = [-sp * x[0] + cp * x[2], 0.0, -cp * x[0] - sp * x[2]];
            let dr = [cp * dx[0] + sp * dx[2], dx[1], -sp * dx[0] + cp * dx[2]];
            [dot3(n, dp), dot3(n, dr)]
        })
    }
    /// Joint torques (pitch, roll) from actuator forces F (N, + = extending): τ = Jᵀ F.
    pub fn torque(&self, p: f64, r: f64, f: [f64; 2]) -> [f64; 2] {
        let j = self.jac(p, r);
        [j[0][0] * f[0] + j[1][0] * f[1], j[0][1] * f[0] + j[1][1] * f[1]]
    }
    /// Actuator forces for joint torques: F = J⁻ᵀ τ. `None` when singular.
    pub fn forces(&self, p: f64, r: f64, tau: [f64; 2]) -> Option<[f64; 2]> {
        let j = self.jac(p, r);
        let det = det2(&j);
        if det.abs() < 1e-15 {
            return None;
        }
        // Solve Jᵀ F = τ.
        Some([(j[1][1] * tau[0] - j[1][0] * tau[1]) / det, (-j[0][1] * tau[0] + j[0][0] * tau[1]) / det])
    }
    /// |det J| normalised by the neutral-pose value c · w.
    pub fn det_norm(&self, p: f64, r: f64) -> f64 {
        det2(&self.jac(p, r)).abs() / (self.c * self.w).abs().max(1e-12)
    }
    /// Largest pure torque about joint `axis` (0 pitch, 1 roll) with |F_i| ≤ 1 N each,
    /// i.e. torque per newton of actuator force (m). 0 when singular.
    pub fn arm(&self, p: f64, r: f64, axis: usize) -> f64 {
        let mut e = [0.0; 2];
        e[axis] = 1.0;
        match self.forces(p, r, e) {
            Some(f) => {
                let m = f[0].abs().max(f[1].abs());
                if m > 0.0 && m.is_finite() { 1.0 / m } else { 0.0 }
            }
            None => 0.0,
        }
    }
    /// Largest actuator speed needed per unit joint speed about `axis` (m/rad).
    pub fn speed_arm(&self, p: f64, r: f64, axis: usize) -> f64 {
        let j = self.jac(p, r);
        j[0][axis].abs().max(j[1][axis].abs())
    }
}

fn det2(j: &[[f64; 2]; 2]) -> f64 {
    j[0][0] * j[1][1] - j[0][1] * j[1][0]
}

// ───────────────────────────── Unified mechanism ─────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    SliderCrank(SliderCrank),
    FourBar(FourBar),
    Ankle(Ankle),
}

/// A mechanism with its joint range (deg). For the ankle the range is pitch; `roll_range_deg`
/// bounds the workspace grid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mech {
    pub kind: Kind,
    pub q_min_deg: f64,
    pub q_max_deg: f64,
    pub roll_range_deg: f64,
}

/// Mechanism state at one joint angle.
#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub q: f64,
    /// Actuator length(s) (m). Single-rod mechanisms repeat the value.
    pub s: [f64; 2],
    /// Signed ds/dq (m/rad) of each actuator for motion about the main joint axis.
    pub j: [f64; 2],
    /// Joint torque per newton of the (each) actuator's force limit (m).
    pub h_force: f64,
    /// Peak actuator speed per unit joint speed (m/rad).
    pub h_speed: f64,
    /// Transmission angle (deg) where it applies.
    pub mu_deg: Option<f64>,
    pub flag: Flag,
    /// Four-bar: assembly-mode flip / cannot assemble at this pose.
    pub flip: bool,
    /// Ankle: |det J| below the singular threshold.
    pub singular: bool,
}

impl Pose {
    fn dead(q: f64) -> Pose {
        Pose { q, s: [0.0; 2], j: [0.0; 2], h_force: 0.0, h_speed: 0.0, mu_deg: None, flag: Flag::Fail, flip: true, singular: false }
    }
}

impl Mech {
    pub fn actuators(&self) -> usize {
        if matches!(self.kind, Kind::Ankle(_)) { 2 } else { 1 }
    }
    /// Pose at joint angle q (rad); `roll` (rad) only matters for the ankle.
    pub fn pose(&self, q: f64, roll: f64) -> Pose {
        match self.kind {
            Kind::SliderCrank(m) => {
                let (s, j) = (m.length(q), m.jac(q));
                let h = j.abs();
                let mu = m.rod_lever_angle_deg(q);
                let flag = if h < H_FAIL_FRAC * m.r { Flag::Fail } else { grade_mu(mu) };
                Pose { q, s: [s; 2], j: [j; 2], h_force: h, h_speed: h, mu_deg: Some(mu), flag, flip: false, singular: false }
            }
            Kind::FourBar(m) => match m.solve(q) {
                None => Pose::dead(q),
                Some(p) => {
                    let mu = p.mu_out_deg.min(p.mu_in_deg);
                    let flip = p.branch_margin < 0.02;
                    let h = p.j.abs();
                    let flag = if flip || h < 1e-4 { Flag::Fail } else { grade_mu(mu) };
                    Pose { q, s: [p.s; 2], j: [p.j; 2], h_force: h, h_speed: h, mu_deg: Some(mu), flag, flip, singular: false }
                }
            },
            Kind::Ankle(a) => {
                let s = a.lengths(q, roll);
                let j = a.jac(q, roll);
                let singular = a.det_norm(q, roll) < DET_SINGULAR;
                let h_force = a.arm(q, roll, 0);
                let flag = if singular || h_force <= 0.0 { Flag::Fail } else { Flag::Ok };
                Pose { q, s, j: [j[0][0], j[1][0]], h_force, h_speed: a.speed_arm(q, roll, 0), mu_deg: None, flag, flip: false, singular }
            }
        }
    }
    /// `n` (≥ 2) evenly spaced poses over the joint range.
    pub fn sweep(&self, n: usize, roll: f64) -> Vec<Pose> {
        let n = n.max(2);
        (0..n)
            .map(|i| {
                let qd = self.q_min_deg + (self.q_max_deg - self.q_min_deg) * i as f64 / (n - 1) as f64;
                self.pose(qd.to_radians(), roll)
            })
            .collect()
    }
}

// ───────────────────────────── Presets ─────────────────────────────

/// Mechanism presets. Geometry is an own plausible estimate for a ~1.7 m / ~60 kg humanoid —
/// approximate, for comparison; edit to your design.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preset {
    OptimusKnee,
    OptimusAnkle,
    GenericFourBarKnee,
    GenericAnkle,
}

impl Preset {
    pub const ALL: [Preset; 4] = [Preset::OptimusKnee, Preset::OptimusAnkle, Preset::GenericFourBarKnee, Preset::GenericAnkle];
    pub fn name(self) -> &'static str {
        match self {
            Preset::OptimusKnee => "Optimus-style knee (slider-crank)",
            Preset::OptimusAnkle => "Optimus-style dual-rod ankle",
            Preset::GenericFourBarKnee => "Generic four-bar knee",
            Preset::GenericAnkle => "Generic dual-rod ankle (humanoid)",
        }
    }
    pub fn mech(self) -> Mech {
        match self {
            Preset::OptimusKnee => Mech {
                kind: Kind::SliderCrank(SliderCrank { ax: -0.035, ay: 0.30, r: 0.05, q0_deg: 240.0 }),
                q_min_deg: 0.0,
                q_max_deg: 120.0,
                roll_range_deg: 0.0,
            },
            Preset::GenericFourBarKnee => Mech {
                kind: Kind::FourBar(FourBar {
                    ax: -0.03,
                    ay: 0.32,
                    cx: -0.07,
                    cy: 0.105,
                    rc: 0.045,
                    rp: 0.068,
                    delta_deg: 276.0,
                    lc: 0.11,
                    rb: 0.034,
                    q0_deg: 112.0,
                    branch: -1.0,
                }),
                q_min_deg: 0.0,
                q_max_deg: 120.0,
                roll_range_deg: 0.0,
            },
            Preset::OptimusAnkle => Mech {
                kind: Kind::Ankle(Ankle { w: 0.07, h: 0.26, c: 0.05 }),
                q_min_deg: -35.0,
                q_max_deg: 30.0,
                roll_range_deg: 20.0,
            },
            Preset::GenericAnkle => Mech {
                kind: Kind::Ankle(Ankle { w: 0.08, h: 0.22, c: 0.06 }),
                q_min_deg: -40.0,
                q_max_deg: 30.0,
                roll_range_deg: 25.0,
            },
        }
    }
}

/// Unit-rad helper for callers.
pub fn deg(x: f64) -> f64 {
    x * PI / 180.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fd(f: impl Fn(f64) -> f64, x: f64) -> f64 {
        let e = 1e-6;
        (f(x + e) - f(x - e)) / (2.0 * e)
    }

    #[test]
    fn slider_crank_jacobian_matches_finite_difference() {
        let m = SliderCrank { ax: -0.035, ay: 0.30, r: 0.05, q0_deg: 240.0 };
        for i in 0..=120 {
            let q = deg(i as f64);
            let (a, n) = (m.jac(q), fd(|q| m.length(q), q));
            assert!((a - n).abs() < 1e-7, "q {i}: {a} vs {n}");
        }
    }

    #[test]
    fn slider_crank_perpendicular_pose_gives_h_equal_r() {
        // Base straight above the knee: the rod is vertical-ish; find the pose where it is square
        // to the lever by bisection on (B − A)·B = 0, then h must equal r.
        let m = SliderCrank { ax: 0.01, ay: 0.3, r: 0.04, q0_deg: 250.0 };
        let g = |q: f64| {
            let b = m.rod_end(q);
            dot2(sub2(b, m.base()), b)
        };
        let (mut lo, mut hi) = (0.0, deg(150.0));
        assert!(g(lo) * g(hi) < 0.0);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if g(lo) * g(mid) <= 0.0 { hi = mid } else { lo = mid }
        }
        let q = 0.5 * (lo + hi);
        assert!((m.jac(q).abs() - m.r).abs() < 1e-9, "h {} vs r {}", m.jac(q).abs(), m.r);
        assert!((m.rod_lever_angle_deg(q) - 90.0).abs() < 1e-4);
    }

    #[test]
    fn slider_crank_dead_centre_fails() {
        // Lever pointing straight at the base pivot at q = 0: h = 0.
        let m = Mech { kind: Kind::SliderCrank(SliderCrank { ax: 0.0, ay: 0.3, r: 0.05, q0_deg: 90.0 }), q_min_deg: 0.0, q_max_deg: 10.0, roll_range_deg: 0.0 };
        let p = m.pose(0.0, 0.0);
        assert_eq!(p.flag, Flag::Fail);
        assert!(p.h_force.is_finite() && p.h_force < 1e-9);
    }

    fn four_bar() -> FourBar {
        match Preset::GenericFourBarKnee.mech().kind {
            Kind::FourBar(f) => f,
            _ => unreachable!(),
        }
    }

    #[test]
    fn four_bar_loop_closure_and_jacobian() {
        let m = four_bar();
        for i in 0..=240 {
            let q = deg(i as f64 * 0.5);
            let p = m.solve(q).expect("assembles over range");
            let c = [m.cx, m.cy];
            assert!((len2(sub2(p.k, c)) - m.rc).abs() < 1e-9, "crank closure at {q}");
            assert!((len2(sub2(p.k, p.b)) - m.lc).abs() < 1e-9, "coupler closure at {q}");
            assert!((len2(sub2(p.p, c)) - m.rp).abs() < 1e-9);
            let n = fd(|q| m.solve(q).unwrap().s, q);
            assert!((p.j - n).abs() < 1e-6, "J at {q}: {} vs {n}", p.j);
        }
    }

    #[test]
    fn four_bar_transmission_flags() {
        let base = four_bar();
        let mech = |f: FourBar| Mech { kind: Kind::FourBar(f), q_min_deg: 0.0, q_max_deg: 120.0, roll_range_deg: 0.0 };
        // Preset: no fail anywhere in range.
        assert!(mech(base).sweep(121, 0.0).iter().all(|p| p.flag != Flag::Fail));
        // Grade thresholds.
        assert_eq!(grade_mu(25.0), Flag::Ok);
        assert_eq!(grade_mu(15.0), Flag::Warn);
        assert_eq!(grade_mu(5.0), Flag::Fail);
        assert_eq!(grade_mu(f64::NAN), Flag::Fail);
        // A coupler too short to reach: cannot assemble → flip + fail with finite numbers.
        let p = mech(FourBar { lc: 0.01, ..base }).pose(0.3, 0.0);
        assert!(p.flip && p.flag == Flag::Fail && p.h_force.is_finite());
        // Sweep the coupler length: some geometry must give a warn-grade pose and one a near-flip.
        let mut seen_warn = false;
        let mut seen_flip = false;
        for k in 0..200 {
            let lc = 0.03 + 0.0015 * k as f64;
            for p in mech(FourBar { lc, ..base }).sweep(121, 0.0) {
                seen_warn |= p.flag == Flag::Warn;
                seen_flip |= p.flip;
                if p.flag == Flag::Warn {
                    let mu = p.mu_deg.unwrap();
                    assert!((MU_FAIL_DEG..MU_WARN_DEG).contains(&mu));
                }
            }
        }
        assert!(seen_warn && seen_flip);
    }

    fn ankle() -> Ankle {
        Ankle { w: 0.07, h: 0.26, c: 0.05 }
    }

    #[test]
    fn ankle_pure_pitch_is_symmetric() {
        let a = ankle();
        for p in [-0.5, -0.2, 0.0, 0.3] {
            let s = a.lengths(p, 0.0);
            assert!((s[0] - s[1]).abs() < 1e-12);
        }
        let s = a.lengths(0.0, 0.0);
        assert!((s[0] - a.h).abs() < 1e-12);
        // Roll makes them differ in opposite directions.
        let s = a.lengths(0.0, 0.2);
        assert!(s[0] > a.h && s[1] < a.h || s[0] < a.h && s[1] > a.h);
    }

    #[test]
    fn ankle_jacobian_matches_finite_difference() {
        let a = ankle();
        for (p, r) in [(0.0, 0.0), (0.3, -0.2), (-0.5, 0.25), (0.1, 0.4)] {
            let j = a.jac(p, r);
            for i in 0..2 {
                let np = fd(|x| a.lengths(x, r)[i], p);
                let nr = fd(|x| a.lengths(p, x)[i], r);
                assert!((j[i][0] - np).abs() < 1e-8 && (j[i][1] - nr).abs() < 1e-8, "{p},{r}: {j:?}");
            }
        }
        // Neutral pose: pitch arm = c (both rods), roll arm = w/2 (opposite signs).
        let j = a.jac(0.0, 0.0);
        assert!((j[0][0].abs() - a.c).abs() < 1e-12 && (j[0][1].abs() - 0.5 * a.w).abs() < 1e-12);
        assert!(j[0][1] * j[1][1] < 0.0);
    }

    #[test]
    fn ankle_torque_matches_statics_and_virtual_work() {
        let a = ankle();
        for (p, r) in [(0.2, 0.1), (-0.4, -0.15), (0.05, 0.3)] {
            let f = [700.0, -350.0];
            let tau = a.torque(p, r, f);
            // Independent statics: force F_i along the rod (from U to the foot) at the foot pivot,
            // moment about the origin projected on the pitch axis (y) and the moving roll axis Ry(p)·x.
            let mut m = [0.0; 3];
            for i in 0..2 {
                let lo = a.lower(i, p, r);
                let d = sub3(lo, a.upper(i));
                let fv = d.map(|x| x * f[i] / len3(d));
                let mi = cross3(lo, fv);
                m = [m[0] + mi[0], m[1] + mi[1], m[2] + mi[2]];
            }
            let roll_axis = foot_rot(p, 0.0, [1.0, 0.0, 0.0]);
            // Generalised forces: Q_j = Σ F_i · ∂s_i/∂q_j. The rigid-body moment gives the same via
            // ω = ṗ·y + ṙ·Ry(p)x. Sign: rod pushes on the foot, so work on the joint is +F·ds.
            assert!((tau[0] - m[1]).abs() < 1e-9, "pitch {} vs {}", tau[0], m[1]);
            assert!((tau[1] - dot3(m, roll_axis)).abs() < 1e-9, "roll {} vs {}", tau[1], dot3(m, roll_axis));
            // Finite-difference work: F·Δs = τ·Δq for a small random-ish joint step.
            let dq = [1e-6, -0.7e-6];
            let s0 = a.lengths(p, r);
            let s1 = a.lengths(p + dq[0], r + dq[1]);
            let w_act = f[0] * (s1[0] - s0[0]) + f[1] * (s1[1] - s0[1]);
            let w_joint = tau[0] * dq[0] + tau[1] * dq[1];
            assert!((w_act - w_joint).abs() < 1e-9 * 1000.0, "{w_act} vs {w_joint}");
            // Round trip through F = J⁻ᵀ τ.
            let back = a.forces(p, r, tau).unwrap();
            assert!((back[0] - f[0]).abs() < 1e-6 && (back[1] - f[1]).abs() < 1e-6);
        }
    }

    #[test]
    fn ankle_singular_pose_flagged() {
        // Rods attached on the ankle axis (c = 0): no pitch authority → singular everywhere.
        let m = Mech { kind: Kind::Ankle(Ankle { w: 0.07, h: 0.26, c: 0.0 }), q_min_deg: -10.0, q_max_deg: 10.0, roll_range_deg: 10.0 };
        let p = m.pose(0.0, 0.0);
        assert!(p.singular && p.flag == Flag::Fail && p.h_force.is_finite());
        // Healthy preset at neutral is not singular.
        assert!(!Preset::OptimusAnkle.mech().pose(0.0, 0.0).singular);
    }

    #[test]
    fn presets_are_sane_over_range() {
        for pr in Preset::ALL {
            let m = pr.mech();
            for p in m.sweep(121, 0.0) {
                assert!(p.flag != Flag::Fail, "{:?} fails at {:.1}°", pr, p.q.to_degrees());
                assert!(p.h_force.is_finite() && p.h_force > 0.005, "{:?} h {} at {:.1}°", pr, p.h_force, p.q.to_degrees());
                assert!(p.s[0] > 0.05 && p.s[1] > 0.05);
            }
            if let Kind::Ankle(_) = m.kind {
                // Corners of the workspace stay non-singular.
                for (p, r) in [(m.q_min_deg, m.roll_range_deg), (m.q_max_deg, -m.roll_range_deg)] {
                    assert!(!m.pose(deg(p), deg(r)).singular, "{pr:?}");
                }
            }
        }
    }
}
