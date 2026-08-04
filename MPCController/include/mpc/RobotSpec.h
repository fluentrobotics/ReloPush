#pragma once

#include <string>

// Robot geometry/hardware limits and MPC tuning parameters, factored out of
// MPCController/src/main.cpp so they can be shared, unit tested, and
// overridden from a JSON file without touching code.
//
// NOTE: the MPC prediction horizon (mpc::kHorizon, see MpcCore.h) is
// deliberately NOT part of either struct below -- it stays a compile-time
// constant. See config/robot_spec.json's "_doc" field for the rationale.
namespace mpc {

// Robot geometry and hardware limits. Defaults mirror the MARS production
// values (MARS/src/ReloPushSetup.cpp make_relopush_robot,
// MARS/src/AllocationSearch.cpp initialize_entities) plus controller-side
// conventions (max_steer/max_accel/max_v_*) that MARS's RobotMeta does not
// carry.
struct RobotSpec {
    double wheel_base = 0.29;
    double front_length = 0.36;
    double rear_length = 0.12;
    double width = 0.275;

    double min_turning_radius_transit = 1.02;
    double min_turning_radius_transfer = 1.43;

    double speed_transit = 0.2;
    double speed_transfer = 0.15;

    // Physical actuator limit; deliberately larger than the planner-implied
    // atan(wheel_base / min_turning_radius_transit) to leave correction
    // authority for the controller.
    double max_steer = 0.33;
    double max_accel = 0.73;
    double max_v_push = 0.295;
    double max_v_nonpush = 0.38;

    static RobotSpec defaults();

    // Returns "" if the spec is internally consistent (the planner-implied
    // steering angle for the transit turning radius does not exceed the
    // hardware's max_steer). Otherwise returns a human-readable description
    // of the problem.
    std::string validate() const;
};

// MPC solver weights and loop timing.
struct MpcParams {
    double dt = 0.05;
    int control_delay_steps = 1;
    int loop_hz = 20;

    double w_dist = 5.0;
    double w_dist_dirchange = 50.0;
    double w_yaw = 1.0;
    double w_vel = 0.2;
    double w_lat = 40.0;
    double w_control = 0.001;
    double w_delta_rate = 50.0;

    // Published-"speed"-field scaling (see MpcCore.h build_payload). Always
    // applied -- there is no hardware-calibration on/off switch; every
    // invocation of mpc_controller scales the wire "speed" field this way.
    double vel_scale = 1.01;
    double vel_scale_back = 0.9;
    double dir_change_scale = 0.77;

    static MpcParams defaults();
};

// Loads the "robot" and "mpc" top-level objects from a single JSON file,
// overriding only the keys that are present -- callers should seed
// robot_out/mpc_out with defaults() first so absent keys keep their
// compiled defaults. Unknown keys produce a warning on stderr but do not
// fail loading. Throws std::runtime_error if the file cannot be opened or
// is not valid JSON.
void load_spec_from_json(const std::string& path, RobotSpec& robot_out, MpcParams& mpc_out);

} // namespace mpc
