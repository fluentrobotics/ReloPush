// SpeedGovernor.h
//
// Reusable duty-actuated, erpm-feedback speed governor -- extracted out of
// TeleopCore's own "SPEED GOVERNOR MODE" (see TeleopCore.h's class header
// for the full motivating story: FW 2.18's native ERPM mode is dangerous,
// duty-alone requires the operator to manually compensate for battery sag,
// so this closes the loop on MEASURED erpm instead and only ever emits
// SET_DUTY). Driver v2 reuses the SAME governor as one of DriverCore's two
// actuation backends (see DriverCore.h's "actuation" config) -- this class
// is deliberately I/O-free and clock-free (mirrors DriverCore.h/
// TeleopCore.h's own "no wall clock of its own" convention) so both
// callers can drive it with their own synthetic/real time.
//
// Portability: C++14 only -- see VescDriver/CMakeLists.txt's HARD
// PORTABILITY RULES.

#ifndef VESC_DRIVER_SPEED_GOVERNOR_H_
#define VESC_DRIVER_SPEED_GOVERNOR_H_

namespace vesc {

struct SpeedGovernorConfig {
    double kp = 2e-6;              // duty per erpm of (proportional) error.
    double ki = 1e-5;               // duty per erpm-second of (integral) error.
    double ff_gain = 4400.0;        // erpm per (duty*volt) at steady state; <=0 disables the ff term (pure PI).
    double max_duty = 0.15;         // symmetric saturation limit -- also the anti-windup freeze threshold.
    double duty_slew_per_s = 0.1;   // mandatory output slew rate (duty/s) -- ALWAYS applied, not opt-in.
    double erpm_filter_tau_s = 0.1; // EMA time constant (seconds) for the erpm input; alpha = 1-exp(-dt/tau).
};

// PI-with-feedforward duty governor: error = target_erpm - filtered(erpm).
// duty_ff = target_erpm / (ff_gain * max(v_in, 6.0)). Incremental PI: the
// integrator only commits a step when the resulting duty command does NOT
// need clamping to +-max_duty this tick (freeze-on-saturation anti-windup
// -- std::min/std::max introduce no rounding of their own, so an exact
// `==` reliably detects "no clamping occurred"). The output is ALWAYS
// slew-limited toward the (possibly-saturated) duty command by
// duty_slew_per_s, regardless of any caller-side ramp toggle -- this is
// the governor's own built-in gentleness guarantee.
//
// One instance per governed axis. Every method that needs "now"/"dt"
// takes it explicitly -- no internal wall-clock reads -- so tests can
// drive it with synthetic time.
class SpeedGovernor {
   public:
    SpeedGovernor() = default;
    explicit SpeedGovernor(const SpeedGovernorConfig& cfg) { configure(cfg); }

    void configure(const SpeedGovernorConfig& cfg) { config_ = cfg; }
    const SpeedGovernorConfig& config() const { return config_; }

    void set_target_erpm(double target_erpm) { target_erpm_ = target_erpm; }
    double target_erpm() const { return target_erpm_; }

    // Feeds one fresh erpm reading, already in whatever frame the caller
    // wants the governor's target_erpm() to be expressed in (DriverCore/
    // TeleopCore both use VEHICLE frame, converting at their own
    // motor<->vehicle boundary -- this class itself is frame-agnostic).
    // dt_since_last_sample is the elapsed time since the PREVIOUS
    // feed_erpm() call:
    //   - The very FIRST call ever (or the first call after reset())
    //     seeds the filter DIRECTLY at erpm_vehicle, ignoring
    //     dt_since_last_sample entirely (matches TeleopCore's former
    //     "!erpm_filter_initialized_" seeding behavior byte-for-byte).
    //   - Otherwise, dt_since_last_sample <= 0.0 (a repeated timestamp or
    //     a backward clock jump) leaves the filter unchanged -- not a
    //     divide-by-zero, not a full snap.
    //   - Otherwise, alpha = 1 - exp(-dt_since_last_sample / tau) (tau =
    //     config().erpm_filter_tau_s, floored well above 0 internally so
    //     a misconfigured tau<=0 can't divide by zero) and
    //     erpm_filtered_ = alpha*erpm_vehicle + (1-alpha)*erpm_filtered_.
    void feed_erpm(double erpm_vehicle, double dt_since_last_sample);

    // Latest v_in (volts) reading -- NOT reset by reset() (see that
    // method's own doc comment), exactly like TeleopCore's former
    // v_in_last_ ("kept warm" across governor resets).
    void feed_vin(double v_in) { v_in_ = v_in; }
    double v_in() const { return v_in_; }

    // Advances the governor by dt seconds (elapsed since the PREVIOUS
    // step() call, caller-computed) and returns this tick's slew-limited,
    // +-max_duty-clamped duty command -- the only output this class ever
    // produces.
    double step(double dt);

    // Resets the integrator, the erpm filter (next feed_erpm() call
    // reseeds directly, per that method's own doc comment), the
    // saturation flag, and the slew-limited output state to a fresh,
    // never-driven 0 -- callers call this on stop/abort/mode-switch/
    // watchdog-zero, exactly like TeleopCore::reset_governor() before
    // this extraction. Deliberately does NOT touch v_in() (see
    // feed_vin()'s own comment) or target_erpm() (always freshly set by
    // the caller before the next step() anyway).
    void reset();

    // Seeds the slew-limited output state directly, bypassing the
    // slew ramp for exactly this one assignment -- used for kick-end
    // handoff (DriverCore's actuation=="governor" kick bypass emits a
    // fixed kick_duty outside this class, then on kick end calls this so
    // the governor's own subsequent step()s slew smoothly FROM kick_duty
    // rather than jumping from 0).
    void seed_output(double duty) { emitted_duty_ = duty; }

    double erpm_filtered() const { return erpm_filtered_; }
    // True iff the LAST step() call's duty command needed clamping to
    // +-max_duty (i.e. the integrator was frozen that tick) -- exposed
    // for tests/telemetry.
    bool saturated() const { return saturated_; }
    double emitted_duty() const { return emitted_duty_; }

   private:
    SpeedGovernorConfig config_;

    double target_erpm_ = 0.0;
    double v_in_ = 12.0;  // harmless placeholder before the first real reading; step() floors the
                           // EFFECTIVE value used in the ff term at kMinEffectiveVIn regardless.

    double erpm_filtered_ = 0.0;
    bool erpm_filter_initialized_ = false;

    double integrator_ = 0.0;
    bool saturated_ = false;

    double emitted_duty_ = 0.0;
};

}  // namespace vesc

#endif  // VESC_DRIVER_SPEED_GOVERNOR_H_
