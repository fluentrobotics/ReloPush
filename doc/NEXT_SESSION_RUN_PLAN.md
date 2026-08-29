# Run plan for the next hardware session

## State as of 2026-08-28 evening (the earlier "wrong driver config" warning is RESOLVED)

The robot runs the AFFINE driver (ff_mode=affine 0.150 V / 0.404 V·s/m, kick 0.035/250 ms, erpm
threshold 500) rebuilt on the Jetson from this tree's `VescDriver/src`; the repo config and the robot
config are identical (verified by diff before deploy). Live MPC config: `launch_margin` 0.6,
`w_dist` 2.0, `w_lat` 63.0 (H1). Snapshot: `config_snapshots/2026-08-28_validated/*affine*`, `*H1*`.
Findings + results: `MPC/StallAndLaunchDesign.md`.

Deploy procedure for any driver SOURCE change (config-only changes still need the restart):

    bash /Users/jeeho/.claude/jobs/be39e273/tmp/deploy_driver.sh   # rsync src+config, cmake on Jetson, restart, prints startup
    # (script may not survive the job dir; the steps are: rsync VescDriver/{src,tests,config,scripts,CMakeLists.txt,third_party}
    #  robot@192.168.1.12:VescDriver/ ; ssh 'cd VescDriver/build && cmake --build . --target vesc_driver -j4 && bash ../scripts/restart_driver.sh')

Run any trajectory (homes first): `bash $CLAUDE_JOB_DIR/tmp/hw_test.sh <tag> <traj.json> <name> --external-loc-endpoint tcp://127.0.0.1:3270 --home-pos-tol 0.08`
Replay: `bash $CLAUDE_JOB_DIR/tmp/hw_run.sh <tag> --external-loc-endpoint tcp://127.0.0.1:3270 --home-pos-tol 0.08 [--params <overlay>]`
Both need the PERSISTENT bridge: `nohup build-mac/MPC/optitrack_zmq_bridge --map-config MARS/config/mpc_tune_mocap_map.json --loc-port-start 3270 --log-csv <frames.csv> &`
(the harness's own per-run bridge needs ~6 s for MODELDEF but is only given 1 s → "no localization sample within 1s").

Open items, in priority order:
1. DONE: `cap_recovery_accel` fixed the cap; E1 re-A/B (n=3) modestly better on ripple metrics → `max_speed_excess` 0.05 applied.
2. Mocap outages: 15 complete stream pauses of 3.4–6.4 s in ~55 min (durations cluster at ~3.4 s and ~6.3 s —
   periodic, Motive/switch side). MITIGATED: controller auto-holds (mocap_hold_after_s 0.3) and resumes with
   schedule debt; harness --mocap-hold-abort-s 12. Validated with a controlled 4 s bridge freeze (hw_M5).
   Still ask what on the Motive PC / switch fires every few minutes.
3. Homing: final approach overshoots the 5 cm tolerance ~50% of the time; use --home-pos-tol 0.08 or slow the approach.
4. Post-launch overshoot: v_cmd still reaches 0.32–0.35 after cusps; lag p95 ~1.0 s with H1. Candidates: the fixed
   cap (item 1), `min_moving_speed` retune now that the driver holds 0.08–0.10 m/s.

## Preconditions (run these first, in order)

1. Robot on, parked near the CENTRE of the mocap volume (out-of-volume parking caused
   379 tracking LOST/RECOVERED cycles in 52 s last session).
2. `ssh robot@192.168.1.12 'bash VescDriver/scripts/restart_driver.sh'`
   -> expect `RESTART_OK`. This also guarantees `source=ackermann` (a calib-mode leak
   silently kills the normal control path).
3. Mocap health: `bash /Users/jeeho/.claude/jobs/be39e273/tmp/mocap_check.sh`
   -> must print `CLEAN`. If it reports drops, do NOT drive; fix Motive first.
4. `mpc_estop` running, e-stop flag `/tmp/mpc_tune_estop` cleared.
5. Note idle `v_in`. Every duty threshold below is voltage-dependent.

## Job 1 -- FINISH the launch-kick A/B (was in progress when we paused)

Sustain sweep is DONE: sustain duty 0.0190 (=0.169 m/s), breakaway 0.0350 (=0.312 m/s).
Launch-kick trend so far, n=2 per arm, monotonic -- needs 3+ per arm before committing:
    launch_margin 0.5 (live): unplanned 13.0, ripple 23%, posErr 0.0528
    launch_margin 2.6       : unplanned 10.0, ripple 18%, posErr 0.0489
    launch_margin 3.2       : unplanned  8.0, ripple 17%, posErr 0.0465
Candidates: /Users/jeeho/.claude/jobs/be39e273/tmp/lm_2.6.json and lm_3.2.json
Run 2 more of each arm alternating on `replay_full`, then decide. RE-DERIVE the margin if the
battery voltage has moved: launch_margin = duty*ff_gain*v_in/erpm_per_mps/min_moving_speed - 1.

## Job 1b -- sustain sweep (DONE 2026-08-28, kept for reference)

    ./build-mac/MPC/sustain_sweep --dry-run --erpm-per-mps 3900
    ./build-mac/MPC/sustain_sweep --erpm-per-mps 3900

MUST pass --erpm-per-mps 3900. Left to itself the tool reads driver_config.json's
erpm_per_mps, which is a PLACEHOLDER of 4614 -- 18% high -- so the reported
"equivalent commanded speed" would be wrong by that much.

ALSO DEPLOY FIRST (the repo copy is fixed, the robot's is not):
    scp VescDriver/config/driver_config.json robot@192.168.1.12:VescDriver/config/
    ssh robot@192.168.1.12 'bash VescDriver/scripts/restart_driver.sh'
This sets erpm_per_mps 4614 -> 3900 (measured). It does NOT change control while the
governor is feedforward-only, but it fixes the driver's reported v_est, which was reading
~15% low purely because of the placeholder.

Needs a bridge on tcp://127.0.0.1:3260:
    build-mac/MPC/optitrack_zmq_bridge --mocap-config MPC/config/mocap_config.txt &

Measures the lowest duty that keeps an ALREADY-MOVING robot moving. Shuttles direction
each pass, 1.6 m travel cap, aborts on bounds/overspeed/stale-mocap/e-stop.

WHY IT MATTERS: breakaway from rest is duty 0.035 at ~8.0 V loaded, equivalent to a
commanded 0.32 m/s through the velocity map. EVERY speed the plan uses is below that,
including the 0.15 m/s launch target. Sustain will be lower (kinetic < static friction) --
the gap between the two is how much of the plan's speed range is actually reachable, and
it sets how hard the LAUNCH kick must hit.

## Job 2 -- along-track gain A/B (unfinished from last session)

Baseline vs `MPC/config/tune_candidates/H1_alongtrack.json` (w_dist 5.0->2.0 with
w_lat 60->63, so lateral authority is unchanged), alternating, 3 each, on `replay_full`.
Fixed metric: straight cruise (|ref_vel|>0.15 AND |steering|<0.08), plus off-path, posErr,
and the along-track slope. Last attempt died 5/6 on mocap, so this is still open.

Expectation to hold onto: a 5x cut in w_dist moved the sim slope only 34%, because most of
the along-track response is structural (the MPC must reach the reference one horizon ahead,
so the implied correction is ~error/horizon_time). If H1 barely moves the fluctuation, that
is real evidence the oscillation is NOT along-track gain, and I should stop tuning it.

## Job 3 -- validate + enable the mocap pose-jump filter (new, see doc/MOCAP_POSE_FILTER_PLAN.md)

Ships disabled (`pose_filter.enabled: false` in both map-config files). During the next
two-robot session: record raw frames with `optitrack_zmq_bridge --log-csv <path>` (no
`--pose-filter` flag needed -- the filter runs and logs its own accepted/d2/fx/fy/fyaw
columns even while `enabled: false` is what's actually published, as long as `--pose-filter
on` is passed so the extra columns get written); replay the recording through
`pose_filter_replay --csv <path>` to confirm zero false-positive rejects on real two-robot
data (single-robot robot2 data already validated, see the plan doc's "Implementation
status"); if a genuine identity-swap or pose jump shows up, it becomes a fixture under
`MPC/tests/data/`. Only then flip `pose_filter.enabled: true` in the live map-config.

## Measured facts to carry forward

- Breakaway: duty 0.035 @ v_in ~8.0 V loaded (8.10 idle). Below 0.030 the motor produces
  max 10 erpm and 0.000 m/s -- genuinely stationary, verified in the trial CSVs.
  At 0.035 it jumps straight to 0.30 m/s. Sharp stiction threshold, no gentle onset.
- velocity map is CORRECT: clean trials give erpm_per_mps 3838 and 3896 vs the map's 3900.
  motor_calibration's 4128 fit was skewed by trial_008, which drew 20 A and aborted.
- Battery is now ~8.1 V idle vs ~7.5 V the previous session. Feedforward-only means the
  voltage feedforward sets speed, so cross-session absolute comparisons are NOT valid.
  Always run a fresh baseline arm in the same session.
- Config snapshot: `config_snapshots/2026-08-28_validated/` (both machines + restore steps).
  NOTE `kHorizon = 16` lives in SOURCE (MPC/include/mpc/MpcCore.h), not config.

## Standing hazards

- zsh does NOT word-split unquoted expansions: `${2:+--params $2}` becomes ONE argument.
  Pass tool flags explicitly. This silently produced 6 bogus "failed" runs.
- A stray `optitrack_zmq_bridge` holds UDP 1511 and starves the harness's own bridge.
  Check with `lsof -nP -iUDP:1511` before running.
- Do not count processes with `ps | grep -c pattern` -- it matches the grep's own shell.
- Keep ONE metric definition for a whole comparison. Changing the filter mid-session
  manufactured a phantom 25%->36% "regression" that cost 4 hardware runs and an audit.
