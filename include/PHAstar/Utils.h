#ifndef UTILS_H
#define UTILS_H

#include <cmath>

// from OMPL
constexpr double kDubinsTwoPi = 2. * M_PI;
constexpr double kDubinsEps = 1e-6;
constexpr double kDubinsZero = -1e-7;
inline double mod2pi(double x) {
    if (x < 0 && x > kDubinsZero)
        return 0;
    double xm = x - kDubinsTwoPi * floor(x / kDubinsTwoPi);
    if (kDubinsTwoPi - xm < .5 * kDubinsEps)
        xm = 0.;
    return xm;
}

inline double pi_2_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


#endif // UTILS_H
