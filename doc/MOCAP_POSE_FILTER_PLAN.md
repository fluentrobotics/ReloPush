# Plan: rejecting mocap pose jumps before they reach the controllers (2026-08-28, plan only)

## Problem
Motive occasionally emits a rigid-body pose that is wrong for one (rarely a few) frames — a translation and/or
orientation jump — then returns. Every consumer (each `mpc_controller`, the GUI, `mpc_tune`, drive-to-start) takes
the pose verbatim, so one bad frame becomes a velocity spike, a false "ahead/behind" error, a LAUNCH/STOP-SNAP
decision, or an e-stop. Evidence so far is qualitative (observed on the GUI); the 462k-frame raw log recorded
today (`bridge --log-csv`, robot2 only) contains no jump > 2 cm, so they are rare and/or tied to multi-body
sessions (identity confusion between look-alike bodies `mushr1`/`mushr2`).

## Where the filter goes
`optitrack_zmq_bridge` (`MPC/src/optitrack_zmq_bridge.cpp` + pure logic in `MPC/src/OptiTrackCore.cpp`), per rigid
body, between NatNet parsing and ZMQ publishing. One place protects every consumer (all MPCs, GUI, tune harness,
calibration tools) and the filter runs at the full 120 Hz frame rate even though publishing is 30 Hz. The raw
stream stays available for diagnosis (`--publish-raw` → `/robotN/localization_raw`, and the per-frame `--log-csv`
gains filter columns).

## Method — constant-velocity Kalman filter with an innovation (validation) gate
The standard tracking-literature construction (Bar-Shalom & Fortmann, "Tracking and Data Association", ch. 2/3;
same gate used in every JPDA/MHT tracker):

* State per body: `[x, y, vx, vy, ψ, ω]`; constant-velocity process model; measurement `z = [x, y, ψ]` from
  Motive with `ψ` wrapped (innovation computed on the shortest arc).
* Per frame: predict with the frame's own timestamp `t` (the bridge already parses it and publishes it), form
  innovation `ν = z − Hx̂`, covariance `S = HPHᵀ + R`, and the gate `d² = νᵀS⁻¹ν`.
  * `d² ≤ γ` (χ², 3 DoF, 99.5 % → γ = 12.8): normal update; publish the posterior.
  * `d² > γ`: **reject** — do not update; publish the prediction (one frame of coasting ≈ 8 ms) and count it.
* Position and yaw are gated separately as well (2-DoF and 1-DoF χ²) so a yaw-only flip is caught even when
  translation is perfect (a common Motive symptom for symmetric marker sets).
* Hard physical plausibility on top of the statistical gate (belt and braces): reject any frame implying
  `|v| > 1.0 m/s` or `|ω| > 4 rad/s` for these cars regardless of covariances.
* Re-initialisation rule (so the filter can never diverge from reality): if rejections persist for
  `reinit_after_s` (default 0.25 s ≈ 30 frames) **and** the rejected measurements are self-consistent (their own
  frame-to-frame motion passes the plausibility check), snap the state to the measurement. Also re-init after
  any gap ≥ 0.5 s (the mocap outages) — the controller's mocap-hold already covers the gap; the filter must not
  reject the first frame after it because the robot legitimately stopped.
* Tuning (data-driven, robot2 rig): mocap jitter at rest σ ≈ 0.5 mm / 0.1°; use `R_pos = (5 mm)²`,
  `R_yaw = (1°)²` (inflated to cover model error), process noise from accel std 3 m/s², yaw-accel 15 rad/s².
  At 120 Hz that gives an effective position gate ≈ 2 cm and yaw gate ≈ 4° per frame, while a genuine
  0.4 m/s move is 3.3 mm/frame and 1 rad/s is 0.5°/frame — two orders of margin either way.

Why this rather than a median/Hampel filter: a median-of-N filter costs (N−1)/2 frames of latency on *every*
frame and cannot use velocity; the gated KF adds no latency on accepted frames, rejects any-length spikes, and
gives a clean velocity estimate the controllers currently finite-difference themselves. A Hampel pre-filter is
not needed once the gate exists.

## What it cannot fix, and the diagnostic for it
A persistent body **swap** (Motive relabels `mushr1` as `mushr2`) is not a spike: after `reinit_after_s` the filter
will follow the wrong body, exactly as today. Mitigation is Motive-side (asymmetric marker layouts, distinct
marker counts). The bridge will flag it: when a body's measurement is rejected by its own gate but would be
accepted by another body's gate, log `SWAP?` with both names and increment a per-pair counter; `mars_sim_viz`'s
mocap monitor shows reject/swap counters per robot so it is visible live.

## Deliverables
1. `MPC/include/mpc/PoseFilter.h` + `MPC/src/PoseFilter.cpp` (pure, no ZMQ): `PoseFilter::step(z, t) → {pose,
   velocity, accepted, d2_pos, d2_yaw, reinit}`; config struct with the parameters above; unit tests: no-op on
   clean data (posterior within 1 mm of measurement), single-frame 20 cm spike rejected and never appears in the
   output, single-frame 90° yaw flip rejected, sustained genuine step re-initialises after `reinit_after_s`,
   gap ≥ 0.5 s re-initialises immediately, yaw wrap at ±π, timestamps out of order ignored.
2. Bridge integration: per-body filter instance; `mocap_map_config.json` gains a `pose_filter` block
   (`enabled` default **false** until validated, `gate_chi2_pos/yaw`, `max_speed`, `max_yaw_rate`,
   `reinit_after_s`, `r_pos`, `r_yaw`, `q_acc`, `q_yaw_acc`); localization JSON gains additive fields
   `filt: 1`, `rej: <count>` and `vx, vy, w` (consumers ignore unknown keys — verified for mpc_controller and
   CalibClient); `--publish-raw`; stats line and end-of-run summary print per-body reject/reinit/swap counts;
   `--log-csv` gains `accepted, d2_pos, d2_yaw, fx, fy, fyaw`.
3. Offline replay tool `pose_filter_replay --csv <bridge frames csv> [--params …]` that runs the identical
   filter over a recording and prints rejections with timestamps — this is how the gate thresholds are set and
   how a real jump recording is turned into a regression fixture (`MPC/tests/data/mocap_jump_*.csv`).
4. Validation gate before enabling by default: (a) record raw frames (`--log-csv`) through the next two-robot
   session until at least one real jump is captured; (b) replay: every jump rejected, zero rejections on the
   clean 462k-frame log from 2026-08-28 (false-positive budget = 0); (c) hardware A/B on `replay_full`
   (n ≥ 3) with the filter on vs off — tracking metrics must be unchanged within noise (the filter must be
   invisible on clean data); (d) then `enabled: true` in the live map config.
5. Optional follow-up: let `mpc_controller` consume the filter's `vx, vy` instead of finite-differencing —
   a separate A/B, not part of this change.

## Effort / risk
Core + tests ≈ half a day; bridge plumbing + replay tool ≈ half a day; validation is gated on capturing a real
jump. Risk: over-tight gates reject genuine motion at direction reversals (high acceleration) — the χ² margin
above is large, and the replay over the existing clean recording is the guard.

## Implementation status (2026-08-28, this session)

Implemented in full per the plan above; ships **disabled** (`pose_filter.enabled: false` in both map-config
files) pending the validation-gate checklist in "Deliverables" #4.

**What exists**
- `mpc::PoseFilter` (`MPC/include/mpc/PoseFilter.h`, `MPC/src/PoseFilter.cpp`): the gated constant-velocity KF
  exactly as specified above (position/yaw/joint χ² gates, hard plausibility ceiling, persistent-reject and
  post-gap re-init, `enabled=false` pure pass-through, a const `would_accept()` for the swap diagnostic). Uses
  fixed-size Eigen matrices (`Eigen::Matrix<double,6,6>` etc, stack-allocated, no heap traffic in `step()`).
  **One refinement beyond the plan text**: the "hard physical plausibility" check (`|Δpos|/dt > max_speed`)
  turned out to need a measurement-noise margin (`max_speed*dt + 4·r_pos`, similarly for yaw), not the bare
  ratio — see "Numbers on the clean recording" below for why. The statistical gate never needed this (its
  innovation covariance already reflects `R` independent of `dt`); only the separate hard-ceiling rule did.
- `MPC/tests/pose_filter_tests.cpp` (target `pose_filter_tests`): 13 tests, all passing -- the 12 specified
  ((a)-(l): clean straight/cornering/reversal, single/3-frame spikes, yaw flip, sustained-step re-init,
  gap re-init, yaw wrap, non-increasing timestamp, `enabled=false` pass-through, plausibility-alone) plus
  (m) a regression replay of the real fixture below.
- Bridge integration (`optitrack_zmq_bridge.cpp`): one `PoseFilter` per published body (a `std::vector`
  parallel to `robots` in explicit mode; a member of `AutoBodyState` in auto-discovery mode), stepped on
  **every** received NatNet frame (120 Hz) with `frame.timestamp` when the source stream carries one, else
  `t_arrival` — well before the 30 Hz publish downsampler. `--pose-filter on|off` overrides the map-config's
  `pose_filter.enabled`. `--publish-raw` additionally publishes the unfiltered pose on
  `/<name>/localization_raw` (the SAME PUB socket, a second topic). Swap diagnostic implemented: a rejected
  sample is tested against every other body's filter via `would_accept()`; a hit logs
  `[BRIDGE] SWAP? <name> sample fits <other>` (rate-limited to 1/s per ordered pair) and is counted, with
  per-pair totals in the end-of-run summary.
  **Backward-compatibility design choice (not explicit in the plan text above)**: the additive ZMQ JSON keys
  (`filt`, `rej`, `vx`, `vy`, `w`) and the six additive `--log-csv` columns (`accepted,d2_pos,d2_yaw,fx,fy,
  fyaw`, appended at the end) are only emitted when `pose_filter.enabled` is true. `MPC/tests/
  test_optitrack_bridge.cpp`'s existing Parts 1-6 assert an EXACT JSON `size()` (3, or 4 with `"t"`) and an
  EXACT `--log-csv` header string, with no `pose_filter` block in any of their map-configs — gating the new
  fields on `enabled` keeps those assertions passing byte-for-byte unmodified (verified: all 6 parts still
  pass). This also matches the shipped-disabled default: a zero-flag run today produces IDENTICAL wire/CSV
  output to before this feature existed.
- New Part 7 in `test_optitrack_bridge.cpp` (two bodies, `pose_filter.enabled: true` via a synthetic
  `--map-config`, no injected jumps -- `fake_motive` has no jump-injection CLI and is out of this task's file
  scope) verifies the wiring itself: the enabled-log line, every payload gaining `filt/rej/vx/vy/w`, the CSV
  header/column count, and that on this clean synthetic track virtually everything is accepted with the
  filtered pose within 5 cm of the raw pose. All 7 parts pass.
- `pose_filter_replay` (`MPC/src/pose_filter_replay.cpp`): `--csv <bridge --log-csv file> [--map-config path]
  [--body name] [--gate-pos/--gate-yaw/--gate-all/--max-speed/--max-yaw-rate/--reinit-after-s/--gap-reinit-s/
  --r-pos/--r-yaw/--q-acc/--q-yaw-acc] [--inject-spike t,dx,dy,dyaw]`. Reads `t_arrival,robot,planar_x,
  planar_y,planar_yaw` by column NAME (tolerates future column growth), replays each body's own
  chronological subsequence through a fresh `PoseFilter`, prints every reject/reinit with its d² values, and
  a per-body + total summary (frames/rejects/reinits/ignored, max d² on accepted frames, max Δpos between
  accepted frames). `--map-config`'s loaded config always has `enabled` forced true (a replay run exists to
  see what the gate WOULD do).
- `MPC/tests/data/mocap_clean_excerpt.csv`: a 2000-frame (t=0.01-16.67s) excerpt of the real recording below,
  replayed by `pose_filter_tests`' test (m) with an asserted zero rejects.

**Numbers on the clean recording** (`optitrack_zmq_bridge --log-csv`, 2026-08-28, robot2 only, 461,937 frames,
~3900s / ~65 minutes):
- Before the plausibility-margin refinement (raw `|Δpos|/dt > max_speed` rule): **540 rejects**, all with tiny
  d² (well under every χ² gate) — i.e. the STATISTICAL gate was never the problem. Root cause: this recording
  has a highly irregular, sometimes-sub-millisecond inter-arrival pattern (bursts down to ~1-2 ms between
  consecutive samples, vs. the nominal 8.3 ms period), and ordinary millimeter-scale sensor jitter divided by
  a ~1 ms `dt` implies a multi-m/s "speed" that trivially exceeds `max_speed=1.0` even though nothing moved.
  A separate finding along the way: 66,137 rows (~14%) have a non-increasing/duplicate `t_arrival` — this
  turned out to be `--log-csv`'s own default `operator<<` precision (6 significant digits, i.e. only
  millisecond resolution at t~100-900s), not a real duplicate-arrival phenomenon; harmless here (`step()`
  correctly ignores them per spec) but worth a future fix (write `t_arrival` with `std::setprecision(9)`) so
  a from-scratch recording preserves true sub-ms timing for this kind of analysis.
- After adding the `max_speed*dt + 4·r_pos` (`4·r_yaw` for yaw) margin to the plausibility rule (see
  `motion_plausible()` in `PoseFilter.cpp`): **35 rejects, 18 reinits**. Forensic check (grep'd the raw CSV
  around each reject/reinit timestamp): one is a genuine ~30 cm sustained relocation at t≈3713.3-3713.6s (23
  consecutive rejects over 0.38 s, exactly the designed persistent-reject→reinit behavior, then correctly
  resumes); the other 17 reinits and remaining single-frame rejects (d²≈12-63, only modestly over/near the
  gates) line up with visibly real, non-static motion in the raw `planar_x/y` columns at those timestamps —
  i.e. this specific ~65-minute recording is NOT purely static (it appears to span multiple handled/repositioned
  segments), contradicting this plan's original "no jump > 2cm" characterization (which was likely eyeballed,
  not exhaustively checked). **Did not force these to literal zero** by loosening gates further: doing so risks
  missing a real jump, defeating the point. A synthetic 5cm/single-frame spike injected into the same file
  (`--inject-spike 100.0,0.05,0,0`) is still rejected with d²_pos≈74 (vs. gate 9.21) — detection sensitivity is
  intact.
  - The 2000-frame excerpt used for the `pose_filter_tests` regression fixture (t=0.01-16.67s, well before the
    first real motion) replays with **0 rejects, 1 reinit** (the initial sample) — confirmed clean.
  - Final config used throughout: the plan's own original defaults (unchanged) plus the plausibility-margin
    refinement above; no gate/noise-parameter values needed retuning.

**How to run the replay**: `pose_filter_replay --csv <path> [--map-config MPC/config/mocap_map_config.json]
[--body robot2] [--inject-spike <t>,<dx>,<dy>,<dyaw>]`.

**Config keys** (all under a `"pose_filter"` object in `--map-config`, mirroring `PoseFilterConfig` field
names 1:1 — see `MPC/config/mocap_map_config.json`/`MARS/config/mpc_tune_mocap_map.json` for the shipped
block): `enabled` (bool, default false), `gate_chi2_pos`/`gate_chi2_yaw`/`gate_chi2_all`, `max_speed`,
`max_yaw_rate`, `reinit_after_s`, `gap_reinit_s`, `r_pos`, `r_yaw`, `q_acc`, `q_yaw_acc`. `--pose-filter on|off`
overrides `enabled` from the CLI without editing the file.

**Not yet done (see doc/NEXT_SESSION_RUN_PLAN.md's "Job 3")**: validate on a REAL two-robot recording (the
"What it cannot fix" swap scenario needs two bodies to exercise at all) and capture at least one genuine jump
as a fixture; the hardware A/B (filter on vs off, `replay_full`, n≥3) from "Deliverables" #4c; then flip
`enabled: true` in the live map-config.
