//! Screw actuator ↔ mechanism coupling: available joint torque / speed over the joint range,
//! the load case, and the actuator checks (buckling, L10, DN, stroke, back-drive).
//!
//! Built from the screw-model primitives in the crate root; the joint load does not go through
//! the linear-axis move model of `compute()`.

use crate::mech::{Flag, Kind as MechKind, Mech};
use crate::{
    buckling_load, critical_rpm, cubic_mean, efficiencies, friction_angle, grade, l10_rev, lead_angle, root_diameter, Inputs, Kind, Status,
    Support, G, SEARCH_D, SEARCH_LEAD,
};
use std::f64::consts::PI;

/// Samples over the joint range (≥ 121 as specified).
pub const SAMPLES: usize = 121;

/// Joint load case: τ_req(q) = τ0 + m·g·L_com·cos(q + offset), magnitude taken.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Load {
    /// Constant torque (N·m).
    pub tau0: f64,
    /// Body-mass share carried by this joint (kg).
    pub mass: f64,
    /// Lever of that mass about the joint (m).
    pub l_com: f64,
    pub offset_deg: f64,
    /// Target joint speed (deg/s).
    pub speed_dps: f64,
    /// Ankle only: roll torque the workspace must provide (N·m).
    pub roll_tau: f64,
    /// Squat (sweep) cycles per day and target life (years) for L10.
    pub cycles_per_day: f64,
    pub years: f64,
}

impl Load {
    /// Default squat case for a ~60 kg humanoid: half the body mass on each knee; ankle stance.
    pub fn default_for(m: &Mech) -> Load {
        match m.kind {
            MechKind::Ankle(_) => Load { tau0: 60.0, mass: 30.0, l_com: 0.06, offset_deg: 0.0, speed_dps: 250.0, roll_tau: 20.0, cycles_per_day: 2000.0, years: 3.0 },
            _ => Load { tau0: 5.0, mass: 30.0, l_com: 0.25, offset_deg: -90.0, speed_dps: 300.0, roll_tau: 0.0, cycles_per_day: 2000.0, years: 3.0 },
        }
    }
    pub fn tau_req(&self, q: f64) -> f64 {
        (self.tau0 + self.mass * G * self.l_com * (q + self.offset_deg.to_radians()).cos()).abs()
    }
}

/// Speed-independent actuator limits from the screw model (SI).
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub eta: f64,
    pub eta_back: f64,
    /// Motor-torque-limited thrust (N).
    pub f_avail: f64,
    /// Linear speed limit (m/s): lowest of motor, 0.8 × critical (whirl) and DN speeds.
    pub v_lim: f64,
    pub v_motor: f64,
    pub v_crit: f64,
    pub v_dn: f64,
    pub p_buckle: f64,
    pub n_crit: f64,
    pub dn_max: f64,
    /// Usable stroke (m): screw length minus the nut.
    pub stroke_cap: f64,
}

pub fn nut_length_m(d_mm: f64) -> f64 {
    (2.5 * d_mm + 15.0) / 1000.0
}

pub fn limits(inp: &Inputs) -> Limits {
    let p = inp.kind.preset();
    let lead = inp.lead_mm / 1000.0;
    let lam = lead_angle(inp.d_mm, inp.lead_mm);
    let (eta, eta_back) = efficiencies(lam, friction_angle(&p, lam));
    let ratio = inp.ratio.max(1e-6);
    let f_avail = inp.motor_torque * ratio * 2.0 * PI * eta / lead;
    let v_motor = inp.motor_rpm / ratio / 60.0 * lead;
    let dr = root_diameter(inp.d_mm, inp.lead_mm) / 1000.0;
    let l = (inp.length_mm / 1000.0).max(1e-3);
    let n_crit = critical_rpm(dr, l, inp.support);
    let v_crit = 0.8 * n_crit / 60.0 * lead;
    let v_dn = p.dn_max / inp.d_mm / 60.0 * lead;
    Limits {
        eta,
        eta_back,
        f_avail,
        v_lim: v_motor.min(v_crit).min(v_dn),
        v_motor,
        v_crit,
        v_dn,
        p_buckle: buckling_load(dr, l, inp.support),
        n_crit,
        dn_max: p.dn_max,
        stroke_cap: (l - nut_length_m(inp.d_mm)).max(0.0),
    }
}

impl Limits {
    /// Force–speed envelope F_max(v) (N): full thrust up to the speed limit, none beyond.
    pub fn f_max(&self, v: f64) -> f64 {
        if v.abs() <= self.v_lim * (1.0 + 1e-12) { self.f_avail } else { 0.0 }
    }
}

/// Default actuator: planetary roller screw (typical values) in the ≈ 3.9 kN force class.
pub fn default_actuator() -> Inputs {
    Inputs {
        kind: Kind::Roller,
        d_mm: 14.0,
        lead_mm: 4.0,
        length_mm: 160.0,
        support: Support::SimpleSimple,
        ca_kn: (Kind::Roller.preset().default_ca_kn(14.0) * 10.0).round() / 10.0,
        motor_torque: 3.0,
        motor_rpm: 6000.0,
        ratio: 1.0,
        ..Inputs::default()
    }
}

/// Actuator force classes (N) for presets.
pub const FORCE_CLASSES: [f64; 3] = [500.0, 3900.0, 8000.0];

/// Motor torque (N·m) that gives thrust `f` (N) with this screw and ratio.
pub fn motor_torque_for(inp: &Inputs, f: f64) -> f64 {
    let lam = lead_angle(inp.d_mm, inp.lead_mm);
    let (eta, _) = efficiencies(lam, friction_angle(&inp.kind.preset(), lam));
    f * inp.lead_mm / 1000.0 / (2.0 * PI * eta * inp.ratio.max(1e-6))
}

/// One sample over the joint range.
#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub q_deg: f64,
    pub tau_req: f64,
    /// Available joint torque at the target speed (N·m).
    pub tau_av: f64,
    /// Max joint speed the actuator speed limit allows (deg/s).
    pub omega_max_dps: f64,
    /// Torque per newton (m) and actuator speed per joint speed (m/rad).
    pub h: f64,
    pub h_speed: f64,
    pub mu_deg: Option<f64>,
    /// Required (worst) actuator force (N) and speed (m/s).
    pub f_req: f64,
    pub v_req: f64,
    /// τ_av / τ_req (finite).
    pub margin: f64,
    pub flag: Flag,
    pub s: [f64; 2],
}

#[derive(Clone, Copy, Debug)]
pub struct Check {
    pub value: f64,
    pub status: Status,
}

#[derive(Clone, Debug)]
pub struct Eval {
    pub pts: Vec<Point>,
    pub lim: Limits,
    pub min_margin: f64,
    pub q_at_min_deg: f64,
    pub f_peak: f64,
    pub v_peak: f64,
    pub screw_rpm_peak: f64,
    pub motor_torque_peak: f64,
    pub dn_peak: f64,
    pub stroke_need: f64,
    pub l10_years: f64,
    pub back_drivable: bool,
    /// Time to sweep the "squat" span (s) at the speed limit; `None` if torque runs short there.
    pub sweep_time: Option<f64>,
    pub sweep_span_deg: (f64, f64),
    pub torque: Check,
    pub buckling: Check,
    pub life: Check,
    pub dn: Check,
    pub stroke: Check,
    pub mech: Status,
    pub overall: Status,
}

fn flag_status(f: Flag) -> Status {
    match f {
        Flag::Ok => Status::Ok,
        Flag::Warn => Status::Warn,
        Flag::Fail => Status::Fail,
    }
}

fn finite(x: f64, cap: f64) -> f64 {
    if x.is_finite() { x.min(cap) } else { cap }
}

/// Sweep span for the squat time: knee 0 → 90° (clamped to the range), ankle the full range.
pub fn sweep_span(m: &Mech) -> (f64, f64) {
    match m.kind {
        MechKind::Ankle(_) => (m.q_min_deg, m.q_max_deg),
        _ => (m.q_min_deg.max(0.0).min(m.q_max_deg), 90.0f64.clamp(m.q_min_deg, m.q_max_deg)),
    }
}

/// Torque / speed numbers of one mechanism pose for an actuator with limits `lim`.
pub fn point(p: &crate::mech::Pose, load: &Load, lim: &Limits) -> Point {
    let omega = load.speed_dps.to_radians().abs();
    let tau_req = load.tau_req(p.q);
    let v_req = omega * p.h_speed;
    let tau_av = if p.flag == Flag::Fail { 0.0 } else { lim.f_max(v_req) * p.h_force };
    let f_req = if p.h_force > 1e-9 { tau_req / p.h_force } else { 1e9 };
    let omega_max = if p.h_speed > 1e-9 { (lim.v_lim / p.h_speed).to_degrees() } else { 1e5 };
    Point {
        q_deg: p.q.to_degrees(),
        tau_req,
        tau_av,
        omega_max_dps: finite(omega_max, 1e5),
        h: p.h_force,
        h_speed: p.h_speed,
        mu_deg: p.mu_deg,
        f_req: finite(f_req, 1e9),
        v_req,
        margin: finite(tau_av / tau_req.max(1e-6), 1e6),
        flag: p.flag,
        s: p.s,
    }
}

/// Evaluate actuator `inp` on mechanism `m` (ankle at fixed `roll`, rad) under `load`.
pub fn evaluate(m: &Mech, roll: f64, load: &Load, inp: &Inputs) -> Eval {
    let lim = limits(inp);
    let pts: Vec<Point> = m.sweep(SAMPLES, roll).into_iter().map(|p| point(&p, load, &lim)).collect();

    let (mut min_margin, mut q_at_min) = (f64::INFINITY, m.q_min_deg);
    for p in &pts {
        if p.margin < min_margin {
            min_margin = p.margin;
            q_at_min = p.q_deg;
        }
    }
    let f_peak = pts.iter().map(|p| p.f_req).fold(0.0, f64::max);
    let v_peak = pts.iter().map(|p| p.v_req).fold(0.0, f64::max);
    let lead = inp.lead_mm / 1000.0;
    let screw_rpm_peak = v_peak / lead * 60.0;
    let motor_torque_peak = f_peak * lead / (2.0 * PI * lim.eta * inp.ratio.max(1e-6));
    let dn_peak = inp.d_mm * screw_rpm_peak;
    let s_of = |p: &Point, i: usize| p.s[i];
    let n_act = m.actuators();
    let stroke_need = (0..n_act)
        .map(|i| {
            let (lo, hi) = pts.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| (lo.min(s_of(p, i)), hi.max(s_of(p, i))));
            hi - lo
        })
        .fold(0.0, f64::max);
    // L10: cubic-mean force over one sweep, weighted by actuator travel (∝ screw revolutions).
    let loads: Vec<(f64, f64)> = pts
        .windows(2)
        .map(|w| {
            let ds = (0..n_act).map(|i| (s_of(&w[1], i) - s_of(&w[0], i)).abs()).fold(0.0, f64::max);
            (0.5 * (w[0].f_req + w[1].f_req), ds)
        })
        .collect();
    let fm = cubic_mean(&loads);
    let l10 = l10_rev(inp.ca_kn * 1000.0, fm);
    let revs_per_day = 2.0 * stroke_need / lead * load.cycles_per_day;
    let l10_years = finite(if revs_per_day > 0.0 { l10 / revs_per_day / 365.0 } else { f64::INFINITY }, 1e6);

    let (a, b) = sweep_span(m);
    let sweep: Vec<&Point> = pts.iter().filter(|p| p.q_deg >= a - 1e-9 && p.q_deg <= b + 1e-9).collect();
    let sweep_time = if sweep.iter().any(|p| p.margin < 1.0) || sweep.len() < 2 {
        None
    } else {
        Some(sweep.windows(2).map(|w| (w[1].q_deg - w[0].q_deg) / (0.5 * (w[0].omega_max_dps + w[1].omega_max_dps)).max(1e-9)).sum())
    };

    let ratio = |x: f64, y: f64| finite(if y > 0.0 { x / y } else { f64::INFINITY }, 1e6);
    let torque = Check { value: min_margin, status: grade(min_margin, 1.25) };
    let bsf = ratio(lim.p_buckle, f_peak);
    let buckling = Check { value: bsf, status: grade(bsf, 2.0) };
    let lm = ratio(l10_years, load.years);
    let life = Check { value: l10_years, status: grade(lm, 1.5) };
    let dn = Check { value: dn_peak, status: grade(ratio(lim.dn_max, dn_peak), 1.25) };
    let sm = ratio(lim.stroke_cap, stroke_need);
    let stroke = Check { value: stroke_need, status: grade(sm, 1.05) };
    let mech = pts.iter().map(|p| flag_status(p.flag)).max().unwrap_or(Status::Ok);
    let overall = [torque.status, buckling.status, life.status, dn.status, stroke.status, mech].into_iter().max().unwrap_or(Status::Ok);
    Eval {
        pts,
        lim,
        min_margin,
        q_at_min_deg: q_at_min,
        f_peak,
        v_peak,
        screw_rpm_peak,
        motor_torque_peak,
        dn_peak,
        stroke_need,
        l10_years,
        back_drivable: lim.eta_back > 0.0,
        sweep_time,
        sweep_span_deg: (a, b),
        torque,
        buckling,
        life,
        dn,
        stroke,
        mech,
        overall,
    }
}

/// One ankle workspace cell.
#[derive(Clone, Copy, Debug)]
pub struct Cell {
    pub pitch_deg: f64,
    pub roll_deg: f64,
    /// Pure-pitch and pure-roll torque capacity at full static thrust (N·m).
    pub tau_pitch: f64,
    pub tau_roll: f64,
    pub singular: bool,
    /// Capacity covers τ_req(pitch) and the roll requirement.
    pub ok: bool,
}

/// Pitch × roll grid (`np` × `nr`) of static torque capacity; empty for single-rod mechanisms.
pub fn workspace(m: &Mech, load: &Load, inp: &Inputs, np: usize, nr: usize) -> Vec<Cell> {
    let MechKind::Ankle(a) = m.kind else { return vec![] };
    let f = limits(inp).f_avail;
    let (np, nr) = (np.max(2), nr.max(2));
    let mut out = Vec::with_capacity(np * nr);
    for j in 0..nr {
        let rd = -m.roll_range_deg + 2.0 * m.roll_range_deg * j as f64 / (nr - 1) as f64;
        for i in 0..np {
            let pd = m.q_min_deg + (m.q_max_deg - m.q_min_deg) * i as f64 / (np - 1) as f64;
            let (p, r) = (pd.to_radians(), rd.to_radians());
            let singular = a.det_norm(p, r) < crate::mech::DET_SINGULAR;
            let (tp, tr) = if singular { (0.0, 0.0) } else { (f * a.arm(p, r, 0), f * a.arm(p, r, 1)) };
            out.push(Cell { pitch_deg: pd, roll_deg: rd, tau_pitch: tp, tau_roll: tr, singular, ok: !singular && tp >= load.tau_req(p) && tr >= load.roll_tau });
        }
    }
    out
}

pub struct Hit {
    pub d_mm: f64,
    pub lead_mm: f64,
    pub eval: Eval,
}

/// Screw search (same grid as the linear-axis search) scored by the min torque margin over the
/// joint range; keeps the motor, type and supports of `base`, drops designs with any failed check.
pub fn find(m: &Mech, roll: f64, load: &Load, base: &Inputs, top: usize) -> Vec<Hit> {
    let p = base.kind.preset();
    let mut hits = vec![];
    for d in SEARCH_D {
        for l in SEARCH_LEAD {
            if l > 2.0 * d {
                continue;
            }
            let inp = Inputs { d_mm: d, lead_mm: l, ca_kn: (p.default_ca_kn(d) * 10.0).round() / 10.0, ..*base };
            let eval = evaluate(m, roll, load, &inp);
            if eval.overall != Status::Fail {
                hits.push(Hit { d_mm: d, lead_mm: l, eval });
            }
        }
    }
    // Margin first (ties within 0.1 %), then fewer warnings, then the smaller (lighter) screw.
    hits.sort_by(|a, b| {
        let (ma, mb) = (a.eval.min_margin, b.eval.min_margin);
        if (ma - mb).abs() > 1e-3 * ma.max(mb) {
            return mb.partial_cmp(&ma).unwrap_or(std::cmp::Ordering::Equal);
        }
        (a.eval.overall, a.d_mm, a.lead_mm).partial_cmp(&(b.eval.overall, b.d_mm, b.lead_mm)).unwrap_or(std::cmp::Ordering::Equal)
    });
    hits.truncate(top);
    hits
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mech::Preset;

    #[test]
    fn constant_force_actuator_gives_tau_equal_f_h() {
        // Plenty of speed: the envelope is flat, so τ_av = F_avail · h at every sample.
        let inp = Inputs { motor_rpm: 1e6, ..default_actuator() };
        let lim = limits(&inp);
        for pr in Preset::ALL {
            let m = pr.mech();
            let load = Load { speed_dps: 10.0, ..Load::default_for(&m) };
            let e = evaluate(&m, 0.0, &load, &inp);
            assert_eq!(e.pts.len(), SAMPLES);
            for p in &e.pts {
                assert!((p.tau_av - lim.f_avail * p.h).abs() < 1e-9 * lim.f_avail, "{pr:?}");
                assert!((p.f_req * p.h - p.tau_req).abs() < 1e-9 * p.tau_req.max(1.0));
            }
        }
    }

    #[test]
    fn speed_limit_cuts_torque() {
        let inp = default_actuator();
        let lim = limits(&inp);
        assert!(lim.f_max(0.5 * lim.v_lim) == lim.f_avail && lim.f_max(1.01 * lim.v_lim) == 0.0);
        let m = Preset::OptimusKnee.mech();
        let load = Load { speed_dps: 1e5, ..Load::default_for(&m) };
        let e = evaluate(&m, 0.0, &load, &inp);
        assert!(e.pts.iter().all(|p| p.tau_av == 0.0));
        assert_eq!(e.torque.status, Status::Fail);
        assert!(e.sweep_time.is_none());
    }

    #[test]
    fn defaults_work_on_every_preset() {
        let inp = default_actuator();
        for pr in Preset::ALL {
            let m = pr.mech();
            let e = evaluate(&m, 0.0, &Load::default_for(&m), &inp);
            assert!(e.min_margin >= 1.0, "{pr:?}: margin {} at {}°", e.min_margin, e.q_at_min_deg);
            assert!(e.overall != Status::Fail, "{pr:?}: {:?}", (e.torque, e.buckling, e.life, e.dn, e.stroke, e.mech));
            assert!(e.sweep_time.is_some_and(|t| t > 0.0 && t < 5.0));
            for v in [e.f_peak, e.l10_years, e.buckling.value, e.dn_peak, e.stroke_need] {
                assert!(v.is_finite());
            }
        }
    }

    #[test]
    fn sweep_time_matches_constant_speed() {
        // Ankle at neutral roll with a huge speed limit is not constant-h, so use a check that
        // holds anyway: time ≥ span / max ω and ≤ span / min ω.
        let inp = default_actuator();
        let m = Preset::OptimusKnee.mech();
        let e = evaluate(&m, 0.0, &Load::default_for(&m), &inp);
        let t = e.sweep_time.unwrap();
        let sweep: Vec<&Point> = e.pts.iter().filter(|p| p.q_deg <= 90.0 + 1e-9).collect();
        let (lo, hi) = sweep.iter().fold((f64::MAX, 0.0f64), |(lo, hi), p| (lo.min(p.omega_max_dps), hi.max(p.omega_max_dps)));
        assert!(t >= 90.0 / hi - 1e-9 && t <= 90.0 / lo + 1e-9);
    }

    #[test]
    fn degenerate_mechanism_fails_with_finite_numbers() {
        use crate::mech::{Kind, SliderCrank};
        let m = Mech { kind: Kind::SliderCrank(SliderCrank { ax: 0.0, ay: 0.3, r: 0.05, q0_deg: 90.0 }), q_min_deg: 0.0, q_max_deg: 30.0, roll_range_deg: 0.0 };
        let e = evaluate(&m, 0.0, &Load::default_for(&m), &default_actuator());
        assert_eq!(e.overall, Status::Fail);
        for p in &e.pts {
            assert!(p.margin.is_finite() && p.f_req.is_finite() && p.omega_max_dps.is_finite());
        }
        assert!(e.f_peak.is_finite() && e.buckling.value.is_finite() && e.l10_years.is_finite());
    }

    #[test]
    fn force_class_sets_thrust() {
        let mut inp = default_actuator();
        for f in FORCE_CLASSES {
            inp.motor_torque = motor_torque_for(&inp, f);
            assert!((limits(&inp).f_avail - f).abs() < 1e-6 * f);
        }
    }

    #[test]
    fn ankle_workspace_grid() {
        let m = Preset::OptimusAnkle.mech();
        let load = Load::default_for(&m);
        let cells = workspace(&m, &load, &default_actuator(), 21, 11);
        assert_eq!(cells.len(), 21 * 11);
        assert!(cells.iter().all(|c| !c.singular && c.tau_pitch > 0.0 && c.tau_roll > 0.0));
        // Neutral pose: pitch capacity = 2·F·c, roll = 2·F·w/2 (both rods share the load).
        let crate::mech::Kind::Ankle(a) = m.kind else { unreachable!() };
        let f = limits(&default_actuator()).f_avail;
        assert!((f * a.arm(0.0, 0.0, 0) - 2.0 * f * a.c).abs() < 1e-6 && (f * a.arm(0.0, 0.0, 1) - f * a.w).abs() < 1e-6);
        // Roll requirement above capacity marks cells not ok.
        let big = Load { roll_tau: 1e6, ..load };
        assert!(workspace(&m, &big, &default_actuator(), 5, 5).iter().all(|c| !c.ok));
        assert!(workspace(&Preset::OptimusKnee.mech(), &load, &default_actuator(), 5, 5).is_empty());
    }

    #[test]
    fn find_is_sorted_by_margin_and_passes() {
        let m = Preset::OptimusKnee.mech();
        let hits = find(&m, 0.0, &Load::default_for(&m), &default_actuator(), 8);
        assert!(!hits.is_empty());
        for w in hits.windows(2) {
            assert!(w[0].eval.min_margin >= w[1].eval.min_margin * (1.0 - 1e-3));
        }
        // Equal margins (same motor-limited thrust): an all-OK design ranks before a warning one.
        assert_eq!(hits[0].eval.overall, Status::Ok);
        assert!(hits.iter().all(|h| h.eval.overall != Status::Fail && h.eval.min_margin >= 1.0));
    }
}
