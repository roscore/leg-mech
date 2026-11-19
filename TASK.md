# Task: leg-mech — humanoid knee / ankle linkage planner driven by linear actuators (Rust → WASM)

New standalone tool. This repo starts as a copy of the author's earlier screw-actuator tool: the
`sd_core` screw model (thrust/torque/efficiency, back-drive, buckling, whirl, DN, L10) and the `sd_web`
WebGL2 renderer, math, plot helpers are REUSED as libraries. Rename crates to `lm_core` / `lm_web`
and replace the UI: this app is about MECHANISMS, the screw model is the actuator underneath.
Original code only, textbook kinematics/statics. No vendor tables, no copied assets.

## Question it answers
"My humanoid's knee or ankle is driven by a linear actuator through a linkage (the way recent humanoids
do it). Over the joint's whole range, what joint torque and speed do I get, and which screw / lead /
motor makes it work?" Pick a mechanism, then compare several screw actuators on it.

## Mechanisms (`lm_core::mech`, pure Rust, unit-tested)
All planar unless noted; degrees in UI, SI internally. Each maps actuator length s ↔ joint angle q
and gives J(q) = ds/dq (m/rad) analytically, checked against central differences in tests.
1. **Knee: actuator + crank (slider-crank)**. Actuator base pivot on the thigh at (a_x, a_y) from the
   knee axis, rod end on a shank lever of radius r at offset q0. Moment arm h(q) = |J(q)|.
   Range default 0…120°.
2. **Knee: actuator + four-bar** (rod → crank → coupler → shank lever). Closed-form circle
   intersection; transmission angle < 20° warning, < 10° fail; flag assembly-mode flips.
3. **Ankle: two parallel linear actuators** (pitch + roll). Two actuators from shank to foot plate at
   ±w/2 lateral, attachment height H above the ankle axis, foot lever c. (p, r) → (s1, s2) by vector
   loop, 2×2 Jacobian, τ = Jᵀ·F. Pitch–roll workspace grid colored by min available torque, singular
   cells flagged (|det J| small).
Presets (approximate, labelled "approx., for comparison — edit to your design"):
- "Optimus-style knee (slider-crank)" and "Optimus-style dual-rod ankle": geometry is your own
  plausible estimate for a ~1.7 m / ~60 kg humanoid; actuator force classes like publicly presented
  ones (≈ 500 N / 3.9 kN / 8 kN) may be used as force presets.
- "Generic four-bar knee", "Generic dual-rod ankle (humanoid)".
Do not state specs for any other company's robot.

## Actuator coupling
- Actuator = existing screw model (type preset, d, lead, length, supports, motor torque/rpm, ratio).
- For q over the range (≥ 121 samples): F_max(v) from the force–speed envelope →
  τ_av(q) = F·h(q), ω_max(q) = v_max/h(q).
- Load case: τ_req(q) = editable constant + m·g·L_com·cos(q + offset) (default: squat, body-mass share
  per knee), target joint speed (deg/s). Required actuator force = τ_req/h feeds buckling / L10 / DN.
- **Compare**: up to 4 actuator configs side by side on the same mechanism: τ_av(q) curves vs τ_req(q),
  ω_max(q), table (min torque margin and where, peak screw force, buckling SF, L10 years,
  back-drivable, 0→90° squat time). Click a column to edit it. Optional "find actuator" search reusing
  the screw search but scored by the min torque margin over the joint range.

## UI
- Korean / English toggle like the earlier tool. Page title "leg-mech".
- 3D (reuse WebGL2 renderer & look, game-grade: lighting, shadows, direct manipulation): thigh /
  shank / foot as shaded bodies, actuator as the animated screw + nut body between pivots, links.
  Drag the shank (or foot) or use a slider to set q; screw spins and nut slides consistently with s(q).
  Actuator turns red where required force > available. Ankle: drag the foot to set pitch/roll.
  Views: perspective / side / front, plus a "play squat" animation.
- 2D plots (reuse plot.rs): τ_av vs τ_req over q, h(q), transmission angle; ankle workspace heatmap.

## Tests
- slider-crank: J(q) vs finite difference; h at the perpendicular pose equals r.
- four-bar: loop closure < 1e-9 over range; transmission-angle flags.
- ankle: symmetric pure pitch gives s1 = s2; Jacobian vs finite difference; τ = Jᵀ F matches
  virtual-work power balance.
- coupling: τ_av = F·h for a constant-force actuator; keep the reused screw-model tests.

## Delivery
- README.md (ko/en) written fresh for leg-mech: what it does, how to use, the "approximate presets"
  disclaimer, build steps, MIT license.
- Pages sub-path `/leg-mech/` (Trunk.toml already set). Keep CLAUDE.md privacy rules.
- Branch `mech`, one PR to main titled "leg-mech: knee/ankle linkage planner with actuator comparison".
  Run `cargo test --workspace`, `trunk build --release`, `sh tools/privacy_check.sh` before the PR.
