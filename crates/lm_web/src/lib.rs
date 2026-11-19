//! leg-mech web UI: mechanism presets + geometry, load case, up to four screw actuators compared
//! on the same joint, plots, actuator search and the 3D leg view.
pub mod gl;
pub mod math;
pub mod plot;
pub mod view3d;

use lm_core::couple::{self, default_actuator, evaluate, find, limits, motor_torque_for, workspace, Cell, Eval, Hit, Load, FORCE_CLASSES};
use lm_core::mech::{Ankle, Flag, FourBar, Kind as MKind, Mech, Preset, SliderCrank};
use lm_core::{Inputs, Kind, Status, Support};
use std::cell::RefCell;
use std::rc::Rc;
use view3d::{Action, Scene, State};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{Document, Element, Event, HtmlCanvasElement, HtmlElement, HtmlInputElement, HtmlSelectElement, PointerEvent, WheelEvent};

const MAX_ACTS: usize = 4;
const WS_NP: usize = 27;
const WS_NR: usize = 17;
const LETTERS: [&str; 4] = ["A", "B", "C", "D"];

struct App {
    preset: Preset,
    mech: Mech,
    load: Load,
    acts: Vec<Inputs>,
    active: usize,
    evals: Vec<Eval>,
    cells: Vec<Cell>,
    hits: Vec<Hit>,
    q_deg: f64,
    roll_deg: f64,
    playing: bool,
    play_t: f64,
    ko: bool,
    dark: bool,
    last_ts: f64,
    plots_dirty: bool,
    scene: Option<Scene>,
}

thread_local! {
    static APP: RefCell<Option<App>> = const { RefCell::new(None) };
}

fn with<R>(f: impl FnOnce(&mut App) -> R) -> Option<R> {
    APP.with(|a| a.borrow_mut().as_mut().map(f))
}

fn doc() -> Document {
    web_sys::window().unwrap().document().unwrap()
}
/// Element by id. Missing ids are logged (not a panic, which would abort the whole app).
fn try_el(id: &str) -> Option<Element> {
    let e = doc().get_element_by_id(id);
    if e.is_none() {
        web_sys::console::error_1(&format!("leg-mech: missing #{id}").into());
    }
    e
}
fn el(id: &str) -> Element {
    try_el(id).unwrap_or_else(|| doc().create_element("div").unwrap())
}
fn input(id: &str) -> HtmlInputElement {
    el(id).unchecked_into()
}
fn select(id: &str) -> HtmlSelectElement {
    el(id).unchecked_into()
}
fn num(id: &str, fallback: f64) -> f64 {
    let v = input(id).value_as_number();
    if v.is_finite() { v } else { fallback }
}
fn set_num(id: &str, v: f64) {
    input(id).set_value(&format!("{}", (v * 1000.0).round() / 1000.0));
}
fn set_html(id: &str, html: &str) {
    el(id).set_inner_html(html);
}
fn show(id: &str, on: bool) {
    let _ = el(id).unchecked_ref::<HtmlElement>().style().set_property("display", if on { "" } else { "none" });
}
fn on(target: &web_sys::EventTarget, ev: &str, f: impl FnMut(Event) + 'static) {
    let c = Closure::<dyn FnMut(Event)>::new(f);
    let _ = target.add_event_listener_with_callback(ev, c.as_ref().unchecked_ref());
    c.forget();
}
fn on_id(id: &str, ev: &str, f: impl FnMut(Event) + 'static) {
    if let Some(e) = try_el(id) {
        on(&e, ev, f);
    }
}
fn tr(ko: bool, en: &'static str, k: &'static str) -> &'static str {
    if ko { k } else { en }
}
fn cls(s: Status) -> &'static str {
    match s {
        Status::Ok => "ok",
        Status::Warn => "warn",
        Status::Fail => "fail",
    }
}
fn st(ko: bool, s: Status) -> &'static str {
    match s {
        Status::Ok => "OK",
        Status::Warn => tr(ko, "warning", "주의"),
        Status::Fail => tr(ko, "fail", "불가"),
    }
}
fn years(v: f64) -> String {
    if v > 999.0 { ">999".into() } else { format!("{v:.1}") }
}
fn is_ankle(m: &Mech) -> bool {
    matches!(m.kind, MKind::Ankle(_))
}

// ───────────────────────────── Geometry fields ─────────────────────────────

/// (en, ko, value in display units, step). Lengths in mm, angles in deg.
fn geo_fields(m: &Mech) -> Vec<(&'static str, &'static str, f64, f64)> {
    let mm = |x: f64| x * 1000.0;
    match m.kind {
        MKind::SliderCrank(g) => vec![
            ("Base pivot a_x (mm)", "베이스 피벗 a_x (mm)", mm(g.ax), 1.0),
            ("Base pivot a_y (mm)", "베이스 피벗 a_y (mm)", mm(g.ay), 1.0),
            ("Lever radius r (mm)", "레버 반경 r (mm)", mm(g.r), 1.0),
            ("Lever offset q0 (°)", "레버 오프셋 q0 (°)", g.q0_deg, 1.0),
        ],
        MKind::FourBar(g) => vec![
            ("Actuator base a_x (mm)", "액추에이터 베이스 a_x (mm)", mm(g.ax), 1.0),
            ("Actuator base a_y (mm)", "액추에이터 베이스 a_y (mm)", mm(g.ay), 1.0),
            ("Crank pivot c_x (mm)", "크랭크 피벗 c_x (mm)", mm(g.cx), 1.0),
            ("Crank pivot c_y (mm)", "크랭크 피벗 c_y (mm)", mm(g.cy), 1.0),
            ("Crank arm to coupler (mm)", "크랭크 암 → 커플러 (mm)", mm(g.rc), 1.0),
            ("Crank arm to rod (mm)", "크랭크 암 → 로드 (mm)", mm(g.rp), 1.0),
            ("Arm angle δ (°)", "암 사이각 δ (°)", g.delta_deg, 1.0),
            ("Coupler length (mm)", "커플러 길이 (mm)", mm(g.lc), 1.0),
            ("Shank lever radius (mm)", "정강이 레버 반경 (mm)", mm(g.rb), 1.0),
            ("Shank lever offset q0 (°)", "정강이 레버 오프셋 q0 (°)", g.q0_deg, 1.0),
            ("Assembly branch (+1 / −1)", "조립 모드 (+1 / −1)", g.branch, 2.0),
        ],
        MKind::Ankle(g) => vec![
            ("Rod spacing w (mm)", "로드 간격 w (mm)", mm(g.w), 1.0),
            ("Attachment height H (mm)", "부착 높이 H (mm)", mm(g.h), 1.0),
            ("Foot lever c (mm)", "발 레버 c (mm)", mm(g.c), 1.0),
        ],
    }
}

fn render_geo(a: &App) {
    let html: String = geo_fields(&a.mech)
        .iter()
        .enumerate()
        .map(|(i, (en, ko, v, step))| {
            format!("<label><span>{}</span><input id=\"g{i}\" type=\"number\" step=\"{step}\" value=\"{}\"></label>", tr(a.ko, en, ko), (v * 1000.0).round() / 1000.0)
        })
        .collect();
    set_html("geo", &html);
    let n = geo_fields(&a.mech).len();
    for i in 0..n {
        on_id(&format!("g{i}"), "change", |_| {
            with(|a| {
                a.mech = read_mech(&a.mech);
                after_mech_change(a);
            });
        });
    }
}

fn read_mech(old: &Mech) -> Mech {
    let g = |i: usize, fb: f64| num(&format!("g{i}"), fb);
    let m = |i: usize, fb: f64| (g(i, fb * 1000.0) / 1000.0).clamp(-2.0, 2.0);
    let pos = |i: usize, fb: f64| m(i, fb).abs().max(0.002);
    let kind = match old.kind {
        MKind::SliderCrank(o) => MKind::SliderCrank(SliderCrank { ax: m(0, o.ax), ay: m(1, o.ay), r: pos(2, o.r), q0_deg: g(3, o.q0_deg) }),
        MKind::FourBar(o) => MKind::FourBar(FourBar {
            ax: m(0, o.ax),
            ay: m(1, o.ay),
            cx: m(2, o.cx),
            cy: m(3, o.cy),
            rc: pos(4, o.rc),
            rp: pos(5, o.rp),
            delta_deg: g(6, o.delta_deg),
            lc: pos(7, o.lc),
            rb: pos(8, o.rb),
            q0_deg: g(9, o.q0_deg),
            branch: if g(10, o.branch) < 0.0 { -1.0 } else { 1.0 },
        }),
        MKind::Ankle(o) => MKind::Ankle(Ankle { w: pos(0, o.w), h: pos(1, o.h), c: m(2, o.c) }),
    };
    let (lo, hi) = if is_ankle(old) { (-80.0, 80.0) } else { (-30.0, 170.0) };
    let qmin = num("qmin", old.q_min_deg).clamp(lo, hi);
    let qmax = num("qmax", old.q_max_deg).clamp(lo, hi).max(qmin + 1.0);
    Mech { kind, q_min_deg: qmin, q_max_deg: qmax, roll_range_deg: num("rollr", old.roll_range_deg).clamp(1.0, 60.0) }
}

fn write_ranges(m: &Mech) {
    set_num("qmin", m.q_min_deg);
    set_num("qmax", m.q_max_deg);
    set_num("rollr", m.roll_range_deg);
    for id in ["q"] {
        let i = input(id);
        i.set_min(&format!("{}", m.q_min_deg));
        i.set_max(&format!("{}", m.q_max_deg));
    }
    let r = input("roll");
    r.set_min(&format!("{}", -m.roll_range_deg));
    r.set_max(&format!("{}", m.roll_range_deg));
}

// ───────────────────────────── Load + actuator fields ─────────────────────────────

const LOAD_IDS: [&str; 8] = ["tau0", "mass", "lcom", "offset", "speed", "rolltau", "cpd", "years"];

fn write_load(l: &Load) {
    let v = [l.tau0, l.mass, l.l_com * 1000.0, l.offset_deg, l.speed_dps, l.roll_tau, l.cycles_per_day, l.years];
    for (id, v) in LOAD_IDS.iter().zip(v) {
        set_num(id, v);
    }
}

fn read_load(o: &Load) -> Load {
    let c = |id, lo: f64, hi: f64, old: f64| num(id, old).clamp(lo, hi);
    Load {
        tau0: c("tau0", -1e4, 1e4, o.tau0),
        mass: c("mass", 0.0, 1e3, o.mass),
        l_com: c("lcom", -2000.0, 2000.0, o.l_com * 1000.0) / 1000.0,
        offset_deg: c("offset", -360.0, 360.0, o.offset_deg),
        speed_dps: c("speed", 1.0, 1e4, o.speed_dps),
        roll_tau: c("rolltau", 0.0, 1e4, o.roll_tau),
        cycles_per_day: c("cpd", 0.0, 1e7, o.cycles_per_day),
        years: c("years", 0.01, 100.0, o.years),
    }
}

const ACT_IDS: [&str; 7] = ["d", "lead", "len", "ca", "mt", "mrpm", "ratio"];

fn force_class(i: &Inputs) -> usize {
    let f = limits(i).f_avail;
    FORCE_CLASSES.iter().position(|c| (f - c).abs() < 0.01 * c).map_or(0, |k| k + 1)
}

fn write_act(i: &Inputs) {
    select("kind").set_value(&Kind::ALL.iter().position(|k| *k == i.kind).unwrap_or(0).to_string());
    select("support").set_value(&Support::ALL.iter().position(|s| *s == i.support).unwrap_or(0).to_string());
    for (id, v) in ACT_IDS.iter().zip([i.d_mm, i.lead_mm, i.length_mm, i.ca_kn, i.motor_torque, i.motor_rpm, i.ratio]) {
        set_num(id, v);
    }
    select("fclass").set_value(&force_class(i).to_string());
}

fn read_act(old: &Inputs) -> Inputs {
    let c = |id, lo: f64, hi: f64, old: f64| num(id, old).clamp(lo, hi);
    Inputs {
        kind: Kind::ALL[select("kind").value().parse::<usize>().unwrap_or(0).min(3)],
        support: Support::ALL[select("support").value().parse::<usize>().unwrap_or(0).min(3)],
        d_mm: c("d", 4.0, 40.0, old.d_mm),
        lead_mm: c("lead", 0.5, 40.0, old.lead_mm),
        length_mm: c("len", 20.0, 2000.0, old.length_mm),
        ca_kn: c("ca", 0.01, 1e4, old.ca_kn),
        motor_torque: c("mt", 0.001, 1e3, old.motor_torque),
        motor_rpm: c("mrpm", 1.0, 1e5, old.motor_rpm),
        ratio: c("ratio", 0.1, 50.0, old.ratio),
        ..*old
    }
}

fn initial_acts() -> Vec<Inputs> {
    let a = default_actuator();
    let mut b = Inputs { kind: Kind::Ball, d_mm: 16.0, lead_mm: 5.0, ca_kn: (Kind::Ball.preset().default_ca_kn(16.0) * 10.0).round() / 10.0, ..a };
    b.motor_torque = round3(motor_torque_for(&b, FORCE_CLASSES[1]));
    let mut c = Inputs { d_mm: 20.0, lead_mm: 5.0, length_mm: 180.0, motor_rpm: 5000.0, ca_kn: (Kind::Roller.preset().default_ca_kn(20.0) * 10.0).round() / 10.0, ..a };
    c.motor_torque = round3(motor_torque_for(&c, FORCE_CLASSES[2]));
    vec![a, b, c]
}

fn round3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

// ───────────────────────────── Compute + render ─────────────────────────────

fn recompute(a: &mut App) {
    let roll = if is_ankle(&a.mech) { a.roll_deg.to_radians() } else { 0.0 };
    a.evals = a.acts.iter().map(|i| evaluate(&a.mech, roll, &a.load, i)).collect();
    a.cells = workspace(&a.mech, &a.load, &a.acts[a.active], WS_NP, WS_NR);
    a.plots_dirty = true;
    render_tables(a);
}

fn after_mech_change(a: &mut App) {
    a.q_deg = a.q_deg.clamp(a.mech.q_min_deg, a.mech.q_max_deg);
    a.roll_deg = a.roll_deg.clamp(-a.mech.roll_range_deg, a.mech.roll_range_deg);
    write_ranges(&a.mech);
    input("q").set_value(&format!("{}", a.q_deg));
    input("roll").set_value(&format!("{}", a.roll_deg));
    a.hits.clear();
    recompute(a);
}

fn kind_name(ko: bool, k: Kind) -> &'static str {
    match k {
        Kind::Ball => tr(ko, "ball", "볼"),
        Kind::Roller => tr(ko, "roller", "롤러"),
        Kind::TrapBronze => tr(ko, "trap. bronze", "사다리꼴 청동"),
        Kind::TrapPom => tr(ko, "trap. POM", "사다리꼴 POM"),
    }
}

fn render_tables(a: &App) {
    let ko = a.ko;
    let f = plot::fmt;
    let ank = is_ankle(&a.mech);
    // Compare table: one column per actuator.
    let mut head = format!("<tr><th></th>");
    for (i, inp) in a.acts.iter().enumerate() {
        head += &format!(
            "<th data-c=\"{i}\" class=\"col{}\"><span class=\"sw\" style=\"background:{}\"></span>{} · {} {}×{}</th>",
            if i == a.active { " active" } else { "" },
            plot::SERIES[i],
            LETTERS[i],
            kind_name(ko, inp.kind),
            f(inp.d_mm),
            f(inp.lead_mm)
        );
    }
    head += "</tr>";
    let row = |label: &str, cell: &dyn Fn(&Eval, &Inputs) -> (String, Option<Status>)| {
        let mut r = format!("<tr><td>{label}</td>");
        for (i, (e, inp)) in a.evals.iter().zip(&a.acts).enumerate() {
            let (v, s) = cell(e, inp);
            let c = s.map_or("", cls);
            r += &format!("<td data-c=\"{i}\" class=\"{c}{}\">{v}</td>", if i == a.active { " active" } else { "" });
        }
        r + "</tr>"
    };
    let span = a.evals.first().map(|e| e.sweep_span_deg).unwrap_or((0.0, 90.0));
    let time_label = if ank {
        format!("{} {:.0}→{:.0}°", tr(ko, "Pitch sweep time", "피치 스윕 시간"), span.0, span.1)
    } else {
        format!("{} {:.0}→{:.0}°", tr(ko, "Squat time", "스쿼트 시간"), span.0, span.1)
    };
    let per = if ank { tr(ko, " (per rod)", " (로드당)") } else { "" };
    let rows = [
        row(&format!("{}{per}", tr(ko, "Thrust / speed limit", "추력 / 속도 한계")), &|e, _| (format!("{} N / {} mm/s", f(e.lim.f_avail), f(e.lim.v_lim * 1000.0)), None)),
        row(tr(ko, "Min torque margin (at q)", "최소 토크 여유 (위치)"), &|e, _| (format!("×{:.2} @ {:.0}°", e.min_margin.min(999.0), e.q_at_min_deg), Some(e.torque.status))),
        row(&format!("{}{per}", tr(ko, "Peak screw force", "최대 스크루 하중")), &|e, _| (format!("{} N", f(e.f_peak)), None)),
        row(tr(ko, "Peak motor torque", "최대 모터 토크"), &|e, i| (format!("{} / {} N·m", f(e.motor_torque_peak), f(i.motor_torque)), None)),
        row(tr(ko, "Buckling SF", "좌굴 안전율"), &|e, _| (format!("{:.1}", e.buckling.value.min(999.0)), Some(e.buckling.status))),
        row(tr(ko, "L10 life (years)", "L10 수명 (년)"), &|e, _| (years(e.l10_years), Some(e.life.status))),
        row("DN", &|e, _| (format!("{} / {}", f(e.dn_peak), f(e.lim.dn_max)), Some(e.dn.status))),
        row(tr(ko, "Stroke need / usable", "필요 / 가용 스트로크"), &|e, _| (format!("{} / {} mm", f(e.stroke_need * 1000.0), f(e.lim.stroke_cap * 1000.0)), Some(e.stroke.status))),
        row(tr(ko, "Back-drivable", "역구동"), &|e, _| (format!("{} (η⁻ {:.2})", if e.back_drivable { tr(ko, "yes", "가능") } else { tr(ko, "no", "불가") }, e.lim.eta_back.max(0.0)), None)),
        row(&time_label, &|e, _| (e.sweep_time.map_or("—".into(), |t| format!("{t:.2} s")), None)),
        row(tr(ko, "Mechanism", "기구"), &|e, _| (st(ko, e.mech).into(), Some(e.mech))),
        row(tr(ko, "Overall", "종합"), &|e, _| (st(ko, e.overall).into(), Some(e.overall))),
    ];
    set_html("cmp", &format!("<thead>{head}</thead><tbody>{}</tbody>", rows.concat()));
    show("add", a.acts.len() < MAX_ACTS);
    show("del", a.acts.len() > 1);
    el("edit-title").set_text_content(Some(&format!("{} {}", tr(ko, "Actuator", "액추에이터"), LETTERS[a.active])));

    // Mechanism flags.
    let mut notes = vec![];
    if let Some(e) = a.evals.first() {
        let qs = |pred: &dyn Fn(&couple::Point) -> bool| -> Option<(f64, f64)> {
            let v: Vec<f64> = e.pts.iter().filter(|p| pred(p)).map(|p| p.q_deg).collect();
            Some((*v.first()?, *v.last()?))
        };
        let poses = a.mech.sweep(couple::SAMPLES, if ank { a.roll_deg.to_radians() } else { 0.0 });
        let flip: Vec<f64> = poses.iter().filter(|p| p.flip).map(|p| p.q.to_degrees()).collect();
        let sing: Vec<f64> = poses.iter().filter(|p| p.singular).map(|p| p.q.to_degrees()).collect();
        if let (Some(x), Some(y)) = (flip.first(), flip.last()) {
            notes.push(format!("<span class=\"fail\">{} {x:.0}…{y:.0}°</span>", tr(ko, "Assembly-mode flip / cannot assemble at", "조립 모드 전환 / 조립 불가:")));
        }
        if let (Some(x), Some(y)) = (sing.first(), sing.last()) {
            notes.push(format!("<span class=\"fail\">{} {x:.0}…{y:.0}°</span>", tr(ko, "Singular (|det J| small) at", "특이 자세 (|det J| 작음):")));
        }
        if let Some((x, y)) = qs(&|p| p.mu_deg.is_some_and(|m| m < lm_core::mech::MU_FAIL_DEG)) {
            notes.push(format!("<span class=\"fail\">{} {x:.0}…{y:.0}°</span>", tr(ko, "Transmission angle < 10° at", "전달각 < 10°:")));
        } else if let Some((x, y)) = qs(&|p| p.mu_deg.is_some_and(|m| m < lm_core::mech::MU_WARN_DEG)) {
            notes.push(format!("<span class=\"warn\">{} {x:.0}…{y:.0}°</span>", tr(ko, "Transmission angle < 20° at", "전달각 < 20°:")));
        }
        if let Some((x, y)) = qs(&|p| p.flag == Flag::Fail && p.mu_deg.is_some_and(|m| m >= lm_core::mech::MU_FAIL_DEG)) {
            notes.push(format!("<span class=\"fail\">{} {x:.0}…{y:.0}°</span>", tr(ko, "Moment arm near zero (dead centre) at", "모멘트 암 ≈ 0 (사점):")));
        }
        if let Some(mu) = e.pts.iter().filter_map(|p| p.mu_deg).reduce(f64::min) {
            notes.push(format!("{} {mu:.0}°", tr(ko, "Min transmission angle", "최소 전달각")));
        }
        let hmin = e.pts.iter().map(|p| p.h).fold(f64::INFINITY, f64::min);
        let hmax = e.pts.iter().map(|p| p.h).fold(0.0, f64::max);
        notes.push(format!("{} {}…{} mm", if ank { tr(ko, "Pitch torque per N (both rods)", "피치 토크/N (두 로드)") } else { tr(ko, "Moment arm h", "모멘트 암 h") }, f(hmin * 1000.0), f(hmax * 1000.0)));
    }
    set_html("flags", &notes.join(" · "));

    // Search hits.
    let hits: String = a
        .hits
        .iter()
        .enumerate()
        .map(|(i, h)| {
            format!(
                "<tr data-i=\"{i}\"><td class=\"{}\">{} × {}</td><td>×{:.2} @ {:.0}°</td><td>{} N</td><td>{:.1}</td><td>{}</td><td>{}</td></tr>",
                cls(h.eval.overall),
                h.d_mm,
                h.lead_mm,
                h.eval.min_margin.min(999.0),
                h.eval.q_at_min_deg,
                f(h.eval.f_peak),
                h.eval.buckling.value.min(999.0),
                years(h.eval.l10_years),
                h.eval.sweep_time.map_or("—".into(), |t| format!("{t:.2} s"))
            )
        })
        .collect();
    set_html("hits", &hits);
}

/// Numbers at the current joint pose for the active actuator.
fn now(a: &App) -> Option<couple::Point> {
    let roll = if is_ankle(&a.mech) { a.roll_deg.to_radians() } else { 0.0 };
    let lim = limits(a.acts.get(a.active)?);
    Some(couple::point(&a.mech.pose(a.q_deg.to_radians(), roll), &a.load, &lim))
}

fn render_hud(a: &App) {
    let Some(p) = now(a) else { return };
    let ko = a.ko;
    let f = plot::fmt;
    let ank = is_ankle(&a.mech);
    let joint = if ank { format!("{} {:.1}° · {} {:.1}°", tr(ko, "pitch", "피치"), a.q_deg, tr(ko, "roll", "롤"), a.roll_deg) } else { format!("q {:.1}°", a.q_deg) };
    let s = if ank { format!("{} / {} mm", f(p.s[0] * 1000.0), f(p.s[1] * 1000.0)) } else { format!("{} mm", f(p.s[0] * 1000.0)) };
    let mc = if p.margin < 1.0 { "fail" } else if p.margin < 1.25 { "warn" } else { "ok" };
    let hud = format!(
        "{joint}<br>s: <b>{s}</b><br>h: <b>{} mm</b><br>τ req: <b>{} N·m</b><br>τ av ({}): <b>{} N·m</b><br>{}: <b class=\"{mc}\">×{:.2}</b><br>F req: <b>{} N</b><br>v req: <b>{} mm/s</b>",
        f(p.h * 1000.0),
        f(p.tau_req),
        LETTERS[a.active],
        f(p.tau_av),
        tr(ko, "margin", "여유"),
        p.margin.min(999.0),
        if p.f_req >= 1e9 { "—".to_string() } else { f(p.f_req) },
        f(p.v_req * 1000.0)
    );
    set_html("hud", &hud);
    el("qlabel").set_text_content(Some(&joint));
}

fn render_plots(a: &App) {
    if a.evals.is_empty() {
        return;
    }
    let ko = a.ko;
    let colors = if a.dark { &plot::DARK } else { &plot::LIGHT };
    let labels: Vec<String> = a.acts.iter().enumerate().map(|(i, inp)| format!("{} τ_av: {} {}×{}", LETTERS[i], kind_name(ko, inp.kind), plot::fmt(inp.d_mm), plot::fmt(inp.lead_mm))).collect();
    let canvas = |id: &str| -> HtmlCanvasElement { el(id).unchecked_into() };
    plot::torque(&canvas("p-tau"), &a.evals, a.active, a.q_deg, colors, &labels, tr(ko, "τ_req (dashed)", "τ_req (점선)"));
    plot::speed(&canvas("p-speed"), &a.evals, a.active, a.q_deg, a.load.speed_dps, colors, tr(ko, "target joint speed (dashed)", "목표 관절 속도 (점선)"));
    let ank = is_ankle(&a.mech);
    plot::geometry(
        &canvas("p-geo"),
        &a.evals[0],
        a.q_deg,
        colors,
        [if ank { tr(ko, "pitch torque per N (mm)", "피치 토크/N (mm)") } else { tr(ko, "moment arm h (mm)", "모멘트 암 h (mm)") }, tr(ko, "transmission angle (right axis)", "전달각 (오른쪽 축)")],
    );
    if ank {
        plot::heatmap(
            &canvas("p-ws"),
            &a.cells,
            WS_NP,
            WS_NR,
            (a.q_deg, a.roll_deg),
            colors,
            [
                tr(ko, "min(pitch, roll) torque capacity, N·m", "min(피치, 롤) 토크 용량, N·m"),
                tr(ko, "× = below requirement", "× = 요구치 미달"),
                tr(ko, "grey = singular", "회색 = 특이 자세"),
            ],
        );
    }
}

fn apply_lang(ko: bool) {
    let key = if ko { "data-ko" } else { "data-en" };
    let list = doc().query_selector_all("[data-en]").unwrap();
    for i in 0..list.length() {
        if let Some(e) = list.item(i).and_then(|n| n.dyn_into::<Element>().ok()) {
            if let Some(t) = e.get_attribute(key) {
                e.set_text_content(Some(&t));
            }
        }
    }
    doc().document_element().unwrap().set_attribute("lang", if ko { "ko" } else { "en" }).ok();
    el("lang").set_text_content(Some(if ko { "English" } else { "한국어" }));
}

fn set_play_label(a: &App) {
    el("play").set_text_content(Some(if a.playing { tr(a.ko, "Stop", "정지") } else if is_ankle(&a.mech) { tr(a.ko, "Play sweep", "스윕 재생") } else { tr(a.ko, "Play squat", "스쿼트 재생") }));
    el("play").class_list().toggle_with_force("on", a.playing).ok();
}

fn mech_visibility(a: &App) {
    let ank = is_ankle(&a.mech);
    show("roll-wrap", ank);
    show("rollr-row", ank);
    show("rolltau-row", ank);
    show("ws-card", ank);
}

fn load_preset(a: &mut App, p: Preset) {
    a.preset = p;
    a.mech = p.mech();
    a.load = Load::default_for(&a.mech);
    a.q_deg = if is_ankle(&a.mech) { 0.0 } else { 45.0 };
    a.roll_deg = 0.0;
    write_load(&a.load);
    render_geo(a);
    mech_visibility(a);
    after_mech_change(a);
    set_play_label(a);
    if let Some(s) = a.scene.as_mut() {
        s.reset_view(&a.mech);
    }
}

// ───────────────────────────── Frame loop + view ─────────────────────────────

fn frame(ts: f64) {
    with(|a| {
        let dt = ((ts - a.last_ts) / 1000.0).clamp(0.0, 0.1);
        a.last_ts = ts;
        if a.playing {
            let (lo, hi) = couple::sweep_span(&a.mech);
            let period = (2.0 * (hi - lo).abs() / 60.0).max(1.0);
            a.play_t = (a.play_t + dt) % period;
            a.q_deg = lo + (hi - lo) * 0.5 * (1.0 - (std::f64::consts::TAU * a.play_t / period).cos());
            input("q").set_value(&format!("{}", a.q_deg));
            a.plots_dirty = true;
        }
        if a.plots_dirty {
            a.plots_dirty = false;
            render_plots(a);
            render_hud(a);
        }
        let over = now(a).is_some_and(|p| p.margin < 1.0);
        let Some(e) = a.evals.get(a.active) else { return };
        let mut s_range = [(f64::MAX, f64::MIN); 2];
        for p in &e.pts {
            for (k, r) in s_range.iter_mut().enumerate() {
                *r = (r.0.min(p.s[k]), r.1.max(p.s[k]));
            }
        }
        let st = State { mech: &a.mech, q: a.q_deg.to_radians(), roll: a.roll_deg.to_radians(), act: &a.acts[a.active], s_range, over };
        if let Some(s) = a.scene.as_mut() {
            let c: HtmlCanvasElement = el("view").unchecked_into();
            let dpr = web_sys::window().unwrap().device_pixel_ratio();
            let (w, h) = ((c.client_width() as f64 * dpr) as u32, (c.client_height() as f64 * dpr) as u32);
            if w == 0 || h == 0 {
                return;
            }
            if c.width() != w || c.height() != h {
                c.set_width(w);
                c.set_height(h);
            }
            s.frame(w as i32, h as i32, dt as f32, &st);
        }
    });
}

fn start_loop() {
    let cb: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let cb2 = cb.clone();
    *cb.borrow_mut() = Some(Closure::new(move |ts: f64| {
        frame(ts);
        if let Some(c) = cb2.borrow().as_ref() {
            let _ = web_sys::window().unwrap().request_animation_frame(c.as_ref().unchecked_ref());
        }
    }));
    let _ = web_sys::window().unwrap().request_animation_frame(cb.borrow().as_ref().unwrap().as_ref().unchecked_ref());
}

fn canvas_xy(e: &PointerEvent) -> (f32, f32) {
    let r = el("view").get_bounding_client_rect();
    let dpr = web_sys::window().unwrap().device_pixel_ratio();
    (((e.client_x() as f64 - r.left()) * dpr) as f32, ((e.client_y() as f64 - r.top()) * dpr) as f32)
}

fn set_joint(a: &mut App, q: f64, roll: f64) {
    let ank = is_ankle(&a.mech);
    a.q_deg = q.clamp(a.mech.q_min_deg, a.mech.q_max_deg);
    input("q").set_value(&format!("{}", a.q_deg));
    a.playing = false;
    set_play_label(a);
    if ank {
        let r = roll.clamp(-a.mech.roll_range_deg, a.mech.roll_range_deg);
        if (r - a.roll_deg).abs() > 1e-9 {
            a.roll_deg = r;
            input("roll").set_value(&format!("{}", a.roll_deg));
            recompute(a);
        }
    }
    a.plots_dirty = true;
}

fn wire_view() {
    let Some(c) = try_el("view") else { return };
    on(&c, "contextmenu", |e| e.prevent_default());
    on(&c, "pointerdown", |e| {
        let e: PointerEvent = e.unchecked_into();
        let _ = el("view").set_pointer_capture(e.pointer_id());
        let (x, y) = canvas_xy(&e);
        with(|a| if let Some(s) = a.scene.as_mut() { s.down(e.pointer_id(), x, y) });
    });
    on(&c, "pointermove", |e| {
        let e: PointerEvent = e.unchecked_into();
        let (x, y) = canvas_xy(&e);
        with(|a| {
            let Some(s) = a.scene.as_mut() else { return };
            let (act, cursor) = s.moved(e.pointer_id(), x, y, e.buttons());
            let _ = el("view").unchecked_ref::<HtmlElement>().style().set_property("cursor", cursor);
            match act {
                Some(Action::Knee(q)) => set_joint(a, q, 0.0),
                Some(Action::KneeDelta(dq)) => {
                    let q = a.q_deg + dq;
                    set_joint(a, q, 0.0);
                }
                Some(Action::Ankle(dp, dr)) => {
                    let (q, r) = (a.q_deg + dp, a.roll_deg + dr);
                    set_joint(a, q, r);
                }
                None => {}
            }
        });
    });
    for ev in ["pointerup", "pointercancel"] {
        on(&c, ev, |e| {
            let e: PointerEvent = e.unchecked_into();
            with(|a| if let Some(s) = a.scene.as_mut() { s.up(e.pointer_id()) });
        });
    }
    on(&c, "wheel", |e| {
        e.prevent_default();
        let e: WheelEvent = e.unchecked_into();
        with(|a| if let Some(s) = a.scene.as_mut() { s.cam.zoom((e.delta_y() * 0.0015).exp() as f32) });
    });
    on(&c, "dblclick", |_| {
        with(|a| if let Some(s) = a.scene.as_mut() { s.reset_view(&a.mech) });
    });
    on_id("cam", "change", |_| {
        with(|a| {
            if let Some(s) = a.scene.as_mut() {
                s.preset = select("cam").value().parse().unwrap_or(0);
                s.reset_view(&a.mech);
            }
        });
    });
    on_id("play", "click", |_| {
        with(|a| {
            a.playing = !a.playing;
            if a.playing {
                // Start the cosine sweep at the current angle's phase.
                let (lo, hi) = couple::sweep_span(&a.mech);
                let period = (2.0 * (hi - lo).abs() / 60.0).max(1.0);
                let u = ((a.q_deg - lo) / (hi - lo)).clamp(0.0, 1.0);
                a.play_t = (1.0 - 2.0 * u).acos() / std::f64::consts::TAU * period;
            }
            set_play_label(a);
        });
    });
}

fn wire_inputs() {
    on_id("preset", "change", |_| {
        with(|a| {
            let i = select("preset").value().parse::<usize>().unwrap_or(0).min(3);
            load_preset(a, Preset::ALL[i]);
        });
    });
    for id in ["qmin", "qmax", "rollr"] {
        on_id(id, "change", |_| {
            with(|a| {
                a.mech = read_mech(&a.mech);
                after_mech_change(a);
            });
        });
    }
    for id in LOAD_IDS {
        on_id(id, "change", |_| {
            with(|a| {
                a.load = read_load(&a.load);
                a.hits.clear();
                recompute(a);
            });
        });
    }
    for id in ACT_IDS.iter().chain(["kind", "support"].iter()) {
        let id = *id;
        on_id(id, "change", move |_| {
            with(|a| {
                let old = a.acts[a.active];
                let mut n = read_act(&old);
                if id == "kind" || id == "d" {
                    n.ca_kn = (n.kind.preset().default_ca_kn(n.d_mm) * 10.0).round() / 10.0;
                    set_num("ca", n.ca_kn);
                }
                // Keep the chosen force class when the screw changes (motor torque follows).
                let fc = select("fclass").value().parse::<usize>().unwrap_or(0);
                if id != "mt" && fc > 0 {
                    n.motor_torque = round3(motor_torque_for(&n, FORCE_CLASSES[fc - 1]));
                    set_num("mt", n.motor_torque);
                }
                a.acts[a.active] = n;
                select("fclass").set_value(&force_class(&n).to_string());
                recompute(a);
            });
        });
    }
    on_id("fclass", "change", |_| {
        with(|a| {
            let fc = select("fclass").value().parse::<usize>().unwrap_or(0);
            if fc > 0 {
                let n = &mut a.acts[a.active];
                n.motor_torque = round3(motor_torque_for(n, FORCE_CLASSES[fc - 1]));
                set_num("mt", n.motor_torque);
                recompute(a);
            }
        });
    });
    on_id("cmp", "click", |e| {
        let Some(t) = e.target().and_then(|t| t.dyn_into::<Element>().ok()).and_then(|t| t.closest("[data-c]").ok().flatten()) else { return };
        let Some(i) = t.get_attribute("data-c").and_then(|s| s.parse::<usize>().ok()) else { return };
        with(|a| {
            if i < a.acts.len() {
                a.active = i;
                write_act(&a.acts[i]);
                recompute(a);
            }
        });
    });
    on_id("add", "click", |_| {
        with(|a| {
            if a.acts.len() < MAX_ACTS {
                let n = a.acts[a.active];
                a.acts.push(n);
                a.active = a.acts.len() - 1;
                write_act(&n);
                recompute(a);
            }
        });
    });
    on_id("del", "click", |_| {
        with(|a| {
            if a.acts.len() > 1 {
                a.acts.remove(a.active);
                a.active = a.active.min(a.acts.len() - 1);
                write_act(&a.acts[a.active]);
                recompute(a);
            }
        });
    });
    on_id("find", "click", |_| {
        with(|a| {
            let roll = if is_ankle(&a.mech) { a.roll_deg.to_radians() } else { 0.0 };
            a.hits = find(&a.mech, roll, &a.load, &a.acts[a.active], 10);
            render_tables(a);
            if a.hits.is_empty() {
                set_html("hits", &format!("<tr><td colspan=\"6\">{}</td></tr>", tr(a.ko, "No screw of this type passes with this motor — try another type, motor or ratio.", "이 모터로 통과하는 해당 종류 스크루가 없습니다 — 종류, 모터, 감속비를 바꿔 보세요.")));
            }
        });
    });
    on_id("hits", "click", |e| {
        let Some(row) = e.target().and_then(|t| t.dyn_into::<Element>().ok()).and_then(|t| t.closest("tr").ok().flatten()) else { return };
        let Some(i) = row.get_attribute("data-i").and_then(|s| s.parse::<usize>().ok()) else { return };
        with(|a| {
            if let Some(h) = a.hits.get(i) {
                let n = &mut a.acts[a.active];
                n.d_mm = h.d_mm;
                n.lead_mm = h.lead_mm;
                n.ca_kn = (n.kind.preset().default_ca_kn(h.d_mm) * 10.0).round() / 10.0;
                let n = *n;
                write_act(&n);
                recompute(a);
            }
        });
    });
    on_id("q", "input", |_| {
        with(|a| {
            let (q, r) = (num("q", a.q_deg), a.roll_deg);
            set_joint(a, q, r);
        });
    });
    on_id("roll", "input", |_| {
        with(|a| {
            let (q, r) = (a.q_deg, num("roll", a.roll_deg));
            set_joint(a, q, r);
        });
    });
    on_id("lang", "click", |_| {
        with(|a| {
            a.ko = !a.ko;
            apply_lang(a.ko);
            render_geo(a);
            set_play_label(a);
            render_tables(a);
            a.plots_dirty = true;
        });
    });
    on(&web_sys::window().unwrap(), "resize", |_| {
        with(|a| a.plots_dirty = true);
    });
}

#[wasm_bindgen(start)]
pub fn start() {
    let win = web_sys::window().unwrap();
    let ko = win.navigator().language().is_some_and(|l| l.to_lowercase().starts_with("ko"));
    let dark = win.match_media("(prefers-color-scheme: dark)").ok().flatten().is_some_and(|m| m.matches());
    apply_lang(ko);
    let canvas: HtmlCanvasElement = el("view").unchecked_into();
    let scene = gl::Renderer::new(&canvas).and_then(Scene::new).map(|mut s| {
        s.r.theme = if dark { gl::DARK } else { gl::LIGHT };
        s
    });
    if scene.is_none() {
        let _ = el("nogl").unchecked_ref::<HtmlElement>().style().set_property("display", "flex");
        show("cam", false);
    }
    let preset = Preset::OptimusKnee;
    let mech = preset.mech();
    let acts = initial_acts();
    write_act(&acts[0]);
    let app = App {
        preset,
        mech,
        load: Load::default_for(&mech),
        acts,
        active: 0,
        evals: vec![],
        cells: vec![],
        hits: vec![],
        q_deg: 45.0,
        roll_deg: 0.0,
        playing: false,
        play_t: 0.0,
        ko,
        dark,
        last_ts: 0.0,
        plots_dirty: true,
        scene,
    };
    APP.with(|a| *a.borrow_mut() = Some(app));
    with(|a| {
        load_preset(a, preset);
        if let Some(s) = a.scene.as_mut() {
            s.cam.cur = s.cam.goal;
        }
    });
    wire_inputs();
    wire_view();
    start_loop();
}
