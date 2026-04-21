#ifndef PARAMS_H
#define PARAMS_H

#include <cmath>

struct Params
{
    double xy_resolution = 0.1;
    double yaw_resolution = M_PI / 6.0; // 30 degrees
    double time_step = 2.0;
    double time_resolution = time_step;
    double min_x = -0.0;
    double min_y = -1.0;
    double max_x = 5.0;
    double max_y = 5.0;
    double max_steer = M_PI / 6.0; // 30 degrees
    double turn_penalty = 1.25;
    double reverse_penalty = 50.0;
    double switch_penalty = 1.2;
    double wait_penalty = 10.0;
    double max_time = 1000.0;
    int collision_steps = 3;
    double analytic_threshold = 5.0;
    double rs_step_size = 0.2;
    double inflation = 1.0;
    double safety_margin = 0.03;
    double robot_collision_inflation = 1.0;
    double movement_length = 0.0; // Added back for dynamic calculation

    double final_push_distance = 0.025;

    // Boundary mode for robot-only out-of-bounds checks:
    // false: strict corner-based check (default, current behavior)
    // true:  origin-only check (valid if robot reference pose is in bounds)
    bool robot_boundary_origin_only = true;
};

#endif // PARAMS_H
