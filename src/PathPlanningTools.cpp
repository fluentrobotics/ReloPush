
#include <PathPlanningTools.h>

float Constants::normalizeHeadingRad(float t) {
    if (t < 0) {
        t = t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
        return 2.f * M_PI + t;
    }

    return t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
}

Environment::PlanningContext::PlanningContext()
{}

Environment::PlanningContext::PlanningContext(bool use_reverse, float turning_r, float LF_in, float speed_lim) : allow_reverse(use_reverse), turning_radius(turning_r), speed_limit(speed_lim)
{
    deltat = speed_limit / turning_r / 1.5;
    xyResolution = turning_r * deltat;
    yawResolution = deltat;

    LF = LF_in;

    dx.resize(6);
    dy.resize(6);
    dyaw.resize(6);

    update_dx();
    update_dy();
    update_dyaw();
}

void Environment::PlanningContext::update_dx()
{
    dx[0] = turning_radius * deltat;
    dx[1] = turning_radius * sin(deltat);
    dx[2] = turning_radius * sin(deltat);
    dx[3] = -turning_radius * deltat;
    dx[4] = -turning_radius * sin(deltat);
    dx[5] = -turning_radius * sin(deltat);
}
void Environment::PlanningContext::update_dy()
{
    double y_term = turning_radius * (1 - cos(deltat));
    dy[0] = 0;
    dy[1] = -y_term;
    dy[2] = y_term;
    dy[3] = 0;
    dy[4] = -y_term;
    dy[5] = y_term;
}
void Environment::PlanningContext::update_dyaw()
{
    dyaw[0] = 0;
    dyaw[1] = deltat;
    dyaw[2] = -deltat;
    dyaw[3] = 0;
    dyaw[4] = -deltat;
    dyaw[5] = deltat;
}
