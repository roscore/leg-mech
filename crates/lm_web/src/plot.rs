//! Tiny canvas-2D plotting: joint torque available vs required, joint speed limit, moment arm
//! and transmission angle over the joint range, and the ankle pitch–roll workspace heatmap.
use lm_core::couple::{Cell, Eval};
use wasm_bindgen::{JsCast, JsValue};
use web_sys::{CanvasRenderingContext2d as Ctx, HtmlCanvasElement};

pub struct Colors {
    pub fg: &'static str,
    pub grid: &'static str,
    pub mut_: &'static str,
}

pub const DARK: Colors = Colors { fg: "#e4e7ec", grid: "#333a45", mut_: "#9aa3b2" };
pub const LIGHT: Colors = Colors { fg: "#1d2128", grid: "#d5d9e0", mut_: "#5b6472" };
/// One colour per compared actuator (A, B, C, D).
pub const SERIES: [&str; 4] = ["#2f9be0", "#e0533a", "#3fbf6f", "#b06fe0"];
const C_WARN: &str = "#d39a10";
const C_FAIL: &str = "#e0533a";

struct Frame {
    ctx: Ctx,
    x0: f64,
    y0: f64,
    w: f64,
    h: f64,
    xmin: f64,
    xmax: f64,
    ymin: f64,
    ymax: f64,
}

impl Frame {
    fn px(&self, x: f64) -> f64 {
        self.x0 + (x - self.xmin) / (self.xmax - self.xmin) * self.w
    }
    fn py(&self, y: f64) -> f64 {
        self.y0 + self.h - (y - self.ymin) / (self.ymax - self.ymin) * self.h
    }
    fn line(&self, pts: &[(f64, f64)], color: &str, width: f64) {
        let c = &self.ctx;
        c.set_stroke_style_str(color);
        c.set_line_width(width);
        c.begin_path();
        for (i, (x, y)) in pts.iter().enumerate() {
            let (x, y) = (self.px(*x), self.py(y.clamp(self.ymin - (self.ymax - self.ymin), self.ymax + (self.ymax - self.ymin))));
            if i == 0 {
                c.move_to(x, y);
            } else {
                c.line_to(x, y);
            }
        }
        c.stroke();
    }
    fn dashed(&self, pts: &[(f64, f64)], color: &str, width: f64) {
        let dash = js_sys::Array::of2(&JsValue::from(6.0), &JsValue::from(4.0));
        let _ = self.ctx.set_line_dash(&dash);
        self.line(pts, color, width);
        let _ = self.ctx.set_line_dash(&js_sys::Array::new());
    }
    fn text(&self, s: &str, x: f64, y: f64, color: &str) {
        self.ctx.set_fill_style_str(color);
        let _ = self.ctx.fill_text(s, x, y);
    }
    fn cursor(&self, x: f64, col: &Colors) {
        if x >= self.xmin && x <= self.xmax {
            self.line(&[(x, self.ymin), (x, self.ymax)], col.fg, 1.0);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn setup(canvas: &HtmlCanvasElement, xmin: f64, xmax: f64, ymin: f64, ymax: f64, col: &Colors, xlabel: &str, right_pad: f64) -> Option<Frame> {
    let dpr = web_sys::window()?.device_pixel_ratio();
    let (cw, ch) = (canvas.client_width() as f64, canvas.client_height() as f64);
    if cw < 10.0 {
        return None;
    }
    canvas.set_width((cw * dpr) as u32);
    canvas.set_height((ch * dpr) as u32);
    let ctx: Ctx = canvas.get_context("2d").ok()??.dyn_into().ok()?;
    ctx.scale(dpr, dpr).ok()?;
    ctx.set_font("11px system-ui, sans-serif");
    let xmax = if xmax > xmin { xmax } else { xmin + 1.0 };
    let ymax = if ymax > ymin { ymax } else { ymin + 1.0 };
    let f = Frame { ctx, x0: 44.0, y0: 8.0, w: cw - 54.0 - right_pad, h: ch - 32.0, xmin, xmax, ymin, ymax };
    for i in 0..=4 {
        let y = ymin + (ymax - ymin) * i as f64 / 4.0;
        f.line(&[(xmin, y), (xmax, y)], col.grid, 1.0);
        f.text(&fmt(y), 2.0, f.py(y) + 4.0, col.mut_);
        let x = xmin + (xmax - xmin) * i as f64 / 4.0;
        f.line(&[(x, ymin), (x, ymax)], col.grid, 1.0);
        f.text(&fmt(x), f.px(x) - 10.0, ch - 12.0, col.mut_);
    }
    f.text(xlabel, f.x0 + f.w - 80.0, ch - 1.0, col.fg);
    Some(f)
}

pub fn fmt(v: f64) -> String {
    let a = v.abs();
    if !v.is_finite() {
        "—".into()
    } else if a >= 1e5 {
        format!("{:.0}k", v / 1e3)
    } else if a >= 100.0 || a == 0.0 {
        format!("{v:.0}")
    } else if a >= 10.0 {
        format!("{v:.1}")
    } else {
        format!("{v:.2}")
    }
}

fn range(evals: &[Eval]) -> (f64, f64) {
    let p = &evals[0].pts;
    (p.first().map_or(0.0, |p| p.q_deg), p.last().map_or(1.0, |p| p.q_deg))
}

/// τ_av(q) per actuator vs τ_req(q) (dashed), min-margin markers, joint cursor.
pub fn torque(canvas: &HtmlCanvasElement, evals: &[Eval], active: usize, q: f64, col: &Colors, labels: &[String], req: &str) {
    if evals.is_empty() {
        return;
    }
    let (a, b) = range(evals);
    let req_max = evals[0].pts.iter().map(|p| p.tau_req).fold(0.0, f64::max);
    let av_max = evals.iter().flat_map(|e| e.pts.iter().map(|p| p.tau_av)).fold(0.0, f64::max);
    let ymax = (req_max * 2.5).max(av_max.min(req_max * 4.0)).max(1.0) * 1.08;
    let Some(f) = setup(canvas, a, b, 0.0, ymax, col, "q (°) · N·m", 0.0) else { return };
    for (i, e) in evals.iter().enumerate() {
        let pts: Vec<(f64, f64)> = e.pts.iter().map(|p| (p.q_deg, p.tau_av)).collect();
        f.line(&pts, SERIES[i % 4], if i == active { 2.6 } else { 1.5 });
        // Min-margin marker.
        if let Some(p) = e.pts.iter().find(|p| (p.q_deg - e.q_at_min_deg).abs() < 1e-9) {
            f.ctx.set_fill_style_str(SERIES[i % 4]);
            f.ctx.begin_path();
            let _ = f.ctx.arc(f.px(p.q_deg), f.py(p.tau_av.min(ymax)), 4.0, 0.0, std::f64::consts::TAU);
            f.ctx.fill();
        }
    }
    let req_pts: Vec<(f64, f64)> = evals[0].pts.iter().map(|p| (p.q_deg, p.tau_req)).collect();
    f.dashed(&req_pts, col.fg, 2.0);
    f.cursor(q, col);
    for (i, s) in labels.iter().enumerate() {
        f.text(s, f.x0 + 8.0, f.y0 + 12.0 + 13.0 * i as f64, SERIES[i % 4]);
    }
    f.text(req, f.x0 + 8.0, f.y0 + 12.0 + 13.0 * labels.len() as f64, col.fg);
}

/// ω_max(q) per actuator and the target joint speed (dashed).
pub fn speed(canvas: &HtmlCanvasElement, evals: &[Eval], active: usize, q: f64, target: f64, col: &Colors, target_label: &str) {
    if evals.is_empty() {
        return;
    }
    let (a, b) = range(evals);
    let top = evals.iter().flat_map(|e| e.pts.iter().map(|p| p.omega_max_dps)).fold(0.0, f64::max);
    let ymax = top.min(target * 4.0).max(target * 1.3).max(1.0);
    let Some(f) = setup(canvas, a, b, 0.0, ymax, col, "q (°) · °/s", 0.0) else { return };
    for (i, e) in evals.iter().enumerate() {
        let pts: Vec<(f64, f64)> = e.pts.iter().map(|p| (p.q_deg, p.omega_max_dps.min(ymax))).collect();
        f.line(&pts, SERIES[i % 4], if i == active { 2.6 } else { 1.5 });
    }
    f.dashed(&[(a, target), (b, target)], col.fg, 2.0);
    f.cursor(q, col);
    f.text(target_label, f.x0 + 8.0, f.y0 + 12.0, col.fg);
}

/// Moment arm h(q) (mm, left axis) and transmission angle μ(q) (deg, scaled 0…90 to the same box).
pub fn geometry(canvas: &HtmlCanvasElement, e: &Eval, q: f64, col: &Colors, labels: [&str; 2]) {
    let (a, b) = (e.pts.first().map_or(0.0, |p| p.q_deg), e.pts.last().map_or(1.0, |p| p.q_deg));
    let hmax = e.pts.iter().map(|p| p.h * 1000.0).fold(0.0, f64::max).max(1.0) * 1.15;
    let Some(f) = setup(canvas, a, b, 0.0, hmax, col, "q (°) · mm", 28.0) else { return };
    let pts: Vec<(f64, f64)> = e.pts.iter().map(|p| (p.q_deg, p.h * 1000.0)).collect();
    f.line(&pts, SERIES[0], 2.2);
    let has_mu = e.pts.iter().any(|p| p.mu_deg.is_some());
    if has_mu {
        let sc = hmax / 90.0;
        for (lim, c) in [(lm_core::mech::MU_WARN_DEG, C_WARN), (lm_core::mech::MU_FAIL_DEG, C_FAIL)] {
            f.dashed(&[(a, lim * sc), (b, lim * sc)], c, 1.0);
        }
        let mu: Vec<(f64, f64)> = e.pts.iter().map(|p| (p.q_deg, p.mu_deg.unwrap_or(0.0) * sc)).collect();
        f.line(&mu, SERIES[2], 2.0);
        for i in 0..=2 {
            let deg = 45.0 * i as f64;
            f.text(&format!("{deg:.0}°"), f.x0 + f.w + 4.0, f.py(deg * sc) + 4.0, SERIES[2]);
        }
    }
    // Mark poses flagged by the mechanism.
    for p in &e.pts {
        let c = match p.flag {
            lm_core::mech::Flag::Ok => continue,
            lm_core::mech::Flag::Warn => C_WARN,
            lm_core::mech::Flag::Fail => C_FAIL,
        };
        f.ctx.set_fill_style_str(c);
        f.ctx.fill_rect(f.px(p.q_deg) - 1.5, f.y0 + f.h - 5.0, 3.0, 5.0);
    }
    f.cursor(q, col);
    f.text(labels[0], f.x0 + 8.0, f.y0 + 12.0, SERIES[0]);
    if has_mu {
        f.text(labels[1], f.x0 + 8.0, f.y0 + 25.0, SERIES[2]);
    }
}

/// Colour ramp 0…1: dark blue → teal → yellow.
fn ramp(t: f64) -> String {
    let t = t.clamp(0.0, 1.0);
    let stops = [(0.0, [40.0, 50.0, 120.0]), (0.5, [30.0, 150.0, 140.0]), (1.0, [240.0, 220.0, 80.0])];
    let (i, k) = if t <= 0.5 { (0, t / 0.5) } else { (1, (t - 0.5) / 0.5) };
    let (a, b) = (stops[i].1, stops[i + 1].1);
    let c = [0, 1, 2].map(|j| (a[j] + (b[j] - a[j]) * k) as u8);
    format!("rgb({},{},{})", c[0], c[1], c[2])
}

/// Pitch × roll grid of min(pure-pitch, pure-roll) torque capacity. Cells missing the requirement
/// get a red cross, singular cells are grey. Cursor at the current pose.
#[allow(clippy::too_many_arguments)]
pub fn heatmap(canvas: &HtmlCanvasElement, cells: &[Cell], np: usize, nr: usize, cur: (f64, f64), col: &Colors, labels: [&str; 3]) {
    if cells.len() != np * nr || cells.is_empty() {
        return;
    }
    let (pmin, pmax) = (cells[0].pitch_deg, cells[np - 1].pitch_deg);
    let (rmin, rmax) = (cells[0].roll_deg, cells[np * nr - 1].roll_deg);
    let Some(f) = setup(canvas, pmin, pmax, rmin, rmax, col, "pitch (°) · roll (°)", 40.0) else { return };
    let vmin = |c: &Cell| c.tau_pitch.min(c.tau_roll);
    let top = cells.iter().map(vmin).fold(0.0, f64::max).max(1e-9);
    let (cw, chh) = (f.w / (np - 1) as f64, f.h / (nr - 1) as f64);
    for c in cells {
        let (x, y) = (f.px(c.pitch_deg) - 0.5 * cw, f.py(c.roll_deg) - 0.5 * chh);
        let fill = if c.singular { "#777".to_string() } else { ramp(vmin(c) / top) };
        f.ctx.set_fill_style_str(&fill);
        f.ctx.fill_rect(x.max(f.x0), y.max(f.y0), cw.min(f.x0 + f.w - x.max(f.x0)) + 0.5, chh.min(f.y0 + f.h - y.max(f.y0)) + 0.5);
        if !c.ok && !c.singular {
            f.ctx.set_stroke_style_str(C_FAIL);
            f.ctx.set_line_width(1.2);
            f.ctx.begin_path();
            let (cx, cy, r) = (x + 0.5 * cw, y + 0.5 * chh, 0.3 * cw.min(chh));
            f.ctx.move_to(cx - r, cy - r);
            f.ctx.line_to(cx + r, cy + r);
            f.ctx.move_to(cx + r, cy - r);
            f.ctx.line_to(cx - r, cy + r);
            f.ctx.stroke();
        }
    }
    // Colour bar on the right.
    for i in 0..20 {
        let t = i as f64 / 19.0;
        f.ctx.set_fill_style_str(&ramp(t));
        f.ctx.fill_rect(f.x0 + f.w + 8.0, f.y0 + f.h * (1.0 - t) - f.h / 20.0, 10.0, f.h / 19.0 + 0.5);
    }
    f.text(&fmt(top), f.x0 + f.w + 4.0, f.y0 + 8.0, col.mut_);
    f.text("0", f.x0 + f.w + 8.0, f.y0 + f.h + 12.0, col.mut_);
    // Current pose.
    f.ctx.set_stroke_style_str(col.fg);
    f.ctx.set_line_width(2.0);
    f.ctx.begin_path();
    let _ = f.ctx.arc(f.px(cur.0), f.py(cur.1), 5.0, 0.0, std::f64::consts::TAU);
    f.ctx.stroke();
    f.text(labels[0], f.x0 + 4.0, f.y0 + 12.0, "#fff");
    f.text(labels[1], f.x0 + 4.0, f.y0 + 25.0, "#fff");
    f.text(labels[2], f.x0 + 4.0, f.y0 + 38.0, "#fff");
}
