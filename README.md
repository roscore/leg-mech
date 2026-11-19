# leg-mech

**Humanoid knee / ankle linkage planner with screw-actuator comparison.** Runs entirely in the
browser (Rust → WebAssembly). No server, no analytics, no network requests.

[한국어](#한국어) · [English](#english)

---

## English

### What it does

Recent humanoids often drive the knee or the ankle with a linear actuator through a linkage.
leg-mech answers: *over the joint's whole range, what joint torque and speed do I get, and which
screw / lead / motor makes it work?*

1. **Pick a mechanism** and edit its geometry:
   - **Knee: actuator + crank (slider-crank).** Base pivot on the thigh, rod end on a shank lever.
     Moment arm h(q) = |ds/dq|.
   - **Knee: actuator + four-bar** (rod → bell-crank → coupler → shank lever). Closed-form circle
     intersection; transmission angle < 20° is a warning and < 10° a failure; assembly-mode flips
     and poses that cannot assemble are flagged.
   - **Ankle: two parallel linear actuators** (pitch + roll). Vector loop (p, r) → (s1, s2),
     2×2 Jacobian, τ = Jᵀ·F, and a pitch–roll workspace map of the available torque with
     singular poses (small |det J|) flagged.
2. **Set the load case**: τ_req(q) = τ0 + m·g·L·cos(q + offset) (default: a squat with half the
   body mass on each knee), the target joint speed, and the sweeps per day for bearing life.
3. **Compare up to four screw actuators** on the same mechanism. Each uses the built-in screw
   model (ball, planetary roller or trapezoidal screw, typical values): thrust from motor
   torque × ratio × efficiency, speed limit from motor speed, whirl (0.8 × critical speed) and DN.
   Over ≥ 121 samples of the range:
   - τ_av(q) = F_max(v) · h(q) at the target speed, ω_max(q) = v_max / h(q);
   - the required actuator force τ_req / h feeds buckling, L10 life and DN.

   The table shows the minimum torque margin and where it occurs, peak screw force, buckling SF,
   L10 in years, back-drivability, stroke needed vs usable, and the 0→90° squat time
   (full pitch range for the ankle). Click a column to edit that actuator.
4. **Find actuator** searches screw diameter × lead for the selected type and motor and ranks
   the designs that pass every check by their minimum torque margin over the range.

The 3D view shows the thigh, shank and foot, the linkage, and each actuator as a motor, a spinning
screw and a sliding nut whose motion follows s(q). Drag the blue foot (or use the slider) to set
the joint; on the ankle, dragging the foot sets pitch and roll. The actuator turns red where the
required force is more than it can give. Views: perspective / side / front, plus a squat
(or pitch sweep) animation. Plots: τ_av vs τ_req, ω_max, h(q) with the transmission angle, and the
ankle workspace heatmap.

### Presets — approximate, for comparison

The presets **"Optimus-style knee (slider-crank)"**, **"Optimus-style dual-rod ankle"**,
**"Generic four-bar knee"** and **"Generic dual-rod ankle (humanoid)"** use our own plausible
geometry estimates for a ~1.7 m / ~60 kg humanoid. They are **approx., for comparison — edit to
your design**. They are not measurements or specifications of any product. The force classes
offered for the motor (≈ 500 N, 3.9 kN, 8 kN) are round numbers for comparison only.

All screw values (efficiency, load rating per size, DN limits) are generic typical estimates,
not catalogue data. Results are educational estimates from textbook kinematics and statics:
check them against the screw maker's data and real tests before building hardware.

### Conventions

- Knee: origin on the knee axis, x forward, y up; the thigh is fixed and points up, q = 0 is a
  straight leg and flexion q > 0 swings the shank backward.
- Ankle: origin on the ankle axis, x forward, y left, z up; foot orientation R = Ry(pitch)·Rx(roll).
  Upper pivots (−c, ±w/2, H) on the shank, lower pivots (−c, ±w/2, 0) on the foot.
- Degrees in the UI, SI units inside.

### Build

Requirements: Rust (stable), the `wasm32-unknown-unknown` target and [trunk](https://trunkrs.dev).

```sh
rustup target add wasm32-unknown-unknown
cargo test --workspace          # mechanism, coupling and screw-model tests
trunk serve                     # dev server; open the printed URL + /leg-mech/
trunk build --release           # static site in dist/ (served under /leg-mech/)
sh tools/privacy_check.sh       # must print "clean"
```

Code layout: `crates/lm_core` (pure Rust: screw model, `mech` linkages, `couple` actuator
coupling) and `crates/lm_web` (WebAssembly UI, WebGL2 renderer, canvas plots).

### License

MIT — see [LICENSE](LICENSE).

---

## 한국어

### 무엇을 하나요

최근 휴머노이드는 무릎이나 발목을 링크를 거친 리니어 액추에이터로 구동하는 경우가 많습니다.
leg-mech는 다음 질문에 답합니다: *관절 전체 범위에서 어떤 관절 토크와 속도가 나오며, 어떤
스크루 / 리드 / 모터를 써야 하는가?*

1. **기구를 고르고** 형상을 편집합니다.
   - **무릎: 액추에이터 + 크랭크 (슬라이더-크랭크).** 허벅지의 베이스 피벗, 정강이 레버의
     로드 끝. 모멘트 암 h(q) = |ds/dq|.
   - **무릎: 액추에이터 + 4절 링크** (로드 → 벨크랭크 → 커플러 → 정강이 레버). 원 교차의
     닫힌 해, 전달각 20° 미만은 주의, 10° 미만은 불가, 조립 모드 전환과 조립 불가 자세 표시.
   - **발목: 병렬 리니어 액추에이터 2개** (피치 + 롤). 벡터 루프 (p, r) → (s1, s2),
     2×2 자코비안, τ = Jᵀ·F, 가용 토크로 칠한 피치–롤 작업 영역 지도와 특이 자세(|det J| 작음) 표시.
2. **부하 조건을 설정합니다**: τ_req(q) = τ0 + m·g·L·cos(q + 오프셋) (기본값: 무릎당 체중의
   절반을 받는 스쿼트), 목표 관절 속도, 베어링 수명용 일일 스윕 횟수.
3. **같은 기구에서 스크루 액추에이터를 최대 4개까지 비교합니다.** 각 액추에이터는 내장 스크루
   모델(볼, 유성 롤러, 사다리꼴 스크루, 일반값)을 씁니다: 추력은 모터 토크 × 감속비 × 효율,
   속도 한계는 모터 속도, 휘돌림(0.8 × 위험 속도), DN 중 최솟값. 범위 전체 121개 이상 샘플에서
   - 목표 속도에서 τ_av(q) = F_max(v) · h(q), ω_max(q) = v_max / h(q),
   - 필요 액추에이터 하중 τ_req / h로 좌굴, L10 수명, DN을 판정합니다.

   표에는 최소 토크 여유와 그 위치, 최대 스크루 하중, 좌굴 안전율, L10 수명(년), 역구동 여부,
   필요 / 가용 스트로크, 0→90° 스쿼트 시간(발목은 피치 전체 범위)이 나옵니다. 열을 클릭하면 그
   액추에이터를 편집합니다.
4. **액추에이터 찾기**는 선택한 종류와 모터로 스크루 지름 × 리드를 탐색하고, 모든 판정을 통과한
   설계를 관절 범위 내 최소 토크 여유 순으로 보여 줍니다.

3D 화면은 허벅지, 정강이, 발, 링크, 그리고 모터 · 회전하는 스크루 · s(q)를 따라 움직이는
너트로 이루어진 액추에이터를 보여 줍니다. 파란 발을 끌거나 슬라이더로 관절 각도를 정하고,
발목에서는 발을 끌어 피치와 롤을 바꿉니다. 필요 하중이 가용 추력을 넘는 자세에서는 액추에이터가
빨갛게 변합니다. 시점: 원근 / 옆 / 앞, 스쿼트(또는 피치 스윕) 애니메이션. 그래프: τ_av 대 τ_req,
ω_max, h(q)와 전달각, 발목 작업 영역 히트맵.

### 프리셋 — 비교용 근사값

**"Optimus-style knee (slider-crank)"**, **"Optimus-style dual-rod ankle"**,
**"Generic four-bar knee"**, **"Generic dual-rod ankle (humanoid)"** 프리셋은 키 약 1.7 m,
체중 약 60 kg 휴머노이드를 가정한 자체 추정 형상입니다. **근사값, 비교용 — 설계에 맞게
수정하세요.** 어떤 제품의 측정값이나 사양이 아닙니다. 모터 추력 등급(≈ 500 N, 3.9 kN, 8 kN)도
비교를 위한 어림수입니다.

스크루 값(효율, 크기별 정격하중, DN 한계)은 모두 일반적인 추정치이며 카탈로그 데이터가
아닙니다. 결과는 교과서 기구학과 정역학에 따른 교육용 추정치이므로, 하드웨어를 만들기 전에
스크루 제조사 데이터와 실제 시험으로 확인하세요.

### 좌표 규약

- 무릎: 원점은 무릎 축, x 앞, y 위. 허벅지는 고정되어 위를 향하고, q = 0은 편 다리, 굴곡
  q > 0이면 정강이가 뒤로 접힙니다.
- 발목: 원점은 발목 축, x 앞, y 왼쪽, z 위. 발 자세 R = Ry(피치)·Rx(롤). 위 피벗
  (−c, ±w/2, H)는 정강이, 아래 피벗 (−c, ±w/2, 0)은 발.
- UI는 도(°), 내부 계산은 SI 단위.

### 빌드

필요: Rust(stable), `wasm32-unknown-unknown` 타깃, [trunk](https://trunkrs.dev).

```sh
rustup target add wasm32-unknown-unknown
cargo test --workspace          # 기구, 커플링, 스크루 모델 테스트
trunk serve                     # 개발 서버; 출력된 주소 + /leg-mech/
trunk build --release           # dist/에 정적 사이트 (/leg-mech/ 경로)
sh tools/privacy_check.sh       # "clean"이 나와야 함
```

코드 구성: `crates/lm_core` (순수 Rust: 스크루 모델, `mech` 링크, `couple` 액추에이터 커플링),
`crates/lm_web` (WebAssembly UI, WebGL2 렌더러, 캔버스 그래프).

### 라이선스

MIT — [LICENSE](LICENSE) 참고.
