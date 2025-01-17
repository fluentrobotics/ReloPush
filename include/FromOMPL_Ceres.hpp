#ifndef FROMOMPL_CERES_HPP
#define FROMOMPL_CERES_HPP


//#include <cmath>
#include <algorithm>
#include <cassert>
#include <boost/math/constants/constants.hpp>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>

#include <ompl/geometric/planners/rrt/RRT.h>

#include <ceres/ceres.h>

using namespace ompl::base;

//#include <cmath>

// Define Pi if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Templated constants
template <typename T>
constexpr T twopi() {
    return static_cast<T>(2) * T(M_PI);
}

template <typename T>
constexpr T DUBINS_EPS() {
    return static_cast<T>(1e-6);
}

template <typename T>
constexpr T DUBINS_ZERO() {
    return static_cast<T>(-1e-7);
}

// Define Pi
template <typename T>
constexpr T pi() {
    return static_cast<T>(M_PI);
}

// Define half Pi
template <typename T>
constexpr T half_pi() {
    return static_cast<T>(M_PI) / T(2.0);
}

template <typename T>
T mod2pi(const T& x) {
    // Wrap x into [0, 2*pi)
    // We use floor on x/(2*pi) casted to T, but do not cast to double.
    // ceres::floor is available in recent Ceres; if not, use a manual approach.
    const T two_pi = T(2.0 * M_PI);
    return x - two_pi * ceres::floor(x / two_pi);
}

namespace DubinsPathCeres
{
template <typename T>
class Path
{
public:
    T t, p, q;
    T rho; // Turning radius

    // Default constructor initializes to zero
    Path() : t(static_cast<T>(100)), p(static_cast<T>(100)), q(static_cast<T>(100)), rho(static_cast<T>(-1)) {}

    // Parameterized constructor
    Path(T t_in, T p_in, T q_in, T rho_in) : t(t_in), p(p_in), q(q_in), rho(rho_in) {}

    T lengthCost()
    {
        return (t+p+q) *rho;
    }
};
}


// Templated Dubins path functions
namespace fromOMPLCeres
{
using namespace ompl::base;
using namespace DubinsPathCeres;

enum DubinsClass
{
    A11 = 0,
    A12 = 1,
    A13 = 2,
    A14 = 3,
    A21 = 4,
    A22 = 5,
    A23 = 6,
    A24 = 7,
    A31 = 8,
    A32 = 9,
    A33 = 10,
    A34 = 11,
    A41 = 12,
    A42 = 13,
    A43 = 14,
    A44 = 15
};



// Templated mod2pi function
template <typename T>
T mod2pi(T x)
{
    if (x < static_cast<T>(0) && x > DUBINS_ZERO<T>()) //DUBINS_ZERO
        return static_cast<T>(0);
    T xm = x - twopi<T>() * ceres::floor(x / twopi<T>());
    if (T(M_PI*2) - xm < static_cast<T>(0.5) * DUBINS_EPS<T>()) // DUBINS_EPS
        xm = static_cast<T>(0);
    return xm;
}

// Templated Dubins path functions
template <typename T>
DubinsPathCeres::Path<T> dubinsLSL(T d, T alpha, T beta, T rho)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    T tmp = static_cast<T>(2) + d * d - static_cast<T>(2) * (ca * cb + sa * sb - d * (sa - sb));
    if (tmp >= DUBINS_ZERO<T>())
    {
        T theta = ceres::atan2(cb - ca, d + sa - sb);
        T t = mod2pi<T>(-alpha + theta);
        T p = ceres::sqrt(std::max(tmp, static_cast<T>(0)));
        T q = mod2pi<T>(beta - theta);
        assert(ceres::abs(p * ceres::cos(alpha + t) - sa + sb - d) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(ceres::abs(p * ceres::sin(alpha + t) + ca - cb) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(mod2pi<T>(alpha + t + q - beta + static_cast<T>(0.5) * DUBINS_EPS<T>()) < DUBINS_EPS<T>());
        return DubinsPathCeres::Path<T>(t, p, q, rho);
    }
    return DubinsPathCeres::Path<T>(); // Return default path if no valid path exists
}

template <typename T>
DubinsPathCeres::Path<T> dubinsRSR(T d, T alpha, T beta, T rho)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    T tmp = static_cast<T>(2) + d * d - static_cast<T>(2) * (ca * cb + sa * sb - d * (sb - sa));
    if (tmp >= DUBINS_ZERO<T>())
    {
        T theta = ceres::atan2(ca - cb, d - sa + sb);
        T t = mod2pi<T>(alpha - theta);
        T p = ceres::sqrt(std::max(tmp, static_cast<T>(0)));
        T q = mod2pi<T>(-beta + theta);
        assert(ceres::abs(p * ceres::cos(alpha - t) + sa - sb - d) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(ceres::abs(p * ceres::sin(alpha - t) - ca + cb) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(mod2pi<T>(alpha - t - q - beta + static_cast<T>(0.5) * DUBINS_EPS<T>()) < DUBINS_EPS<T>());
        return DubinsPathCeres::Path<T>(t, p, q, rho);
    }
    return DubinsPathCeres::Path<T>(); // Return default path if no valid path exists
}

template <typename T>
DubinsPathCeres::Path<T> dubinsRSL(T d, T alpha, T beta, T rho)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    T tmp = d * d - static_cast<T>(2) + static_cast<T>(2) * (ca * cb + sa * sb - d * (sa + sb));

    if (tmp >= DUBINS_ZERO<T>())
    {
        T p = ceres::sqrt(std::max(tmp, static_cast<T>(0)));
        T theta = ceres::atan2(ca + cb, d - sa - sb) - ceres::atan2(T(2), p);
        T t = mod2pi<T>(alpha - theta);
        T q = mod2pi<T>(beta - theta);
        assert(ceres::abs(p * ceres::cos(alpha - t) - T(2) * ceres::sin(alpha - t) + sa + sb - d) < T(2) * DUBINS_EPS<T>());
        assert(ceres::abs(p * ceres::sin(alpha - t) + T(2) * ceres::cos(alpha - t) - ca - cb) < T(2) * DUBINS_EPS<T>());
        assert(mod2pi<T>(alpha - t + q - beta + T(0.5) * DUBINS_EPS<T>()) < DUBINS_EPS<T>());
        return DubinsPathCeres::Path<T>(t, p, q, rho);
    }

    return DubinsPathCeres::Path<T>(); // Return default path if no valid path exists
}

template <typename T>
DubinsPathCeres::Path<T> dubinsLSR(T d, T alpha, T beta, T rho)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    T tmp = static_cast<T>(-2) + d * d + static_cast<T>(2) * (ca * cb + sa * sb + d * (sa + sb));
    if (tmp >= DUBINS_ZERO<T>())
    {
        T p = ceres::sqrt(std::max(tmp, static_cast<T>(0)));
        T theta = ceres::atan2(-ca - cb, d + sa + sb) - ceres::atan2(static_cast<T>(-2), p);
        T t = mod2pi<T>(-alpha + theta);
        T q = mod2pi<T>(-beta + theta);
        assert(ceres::abs(p * ceres::cos(alpha + t) + static_cast<T>(2) * ceres::sin(alpha + t) - sa - sb - d) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(ceres::abs(p * ceres::sin(alpha + t) - static_cast<T>(2) * ceres::cos(alpha + t) + ca + cb) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(mod2pi<T>(alpha + t - q - beta + static_cast<T>(0.5) * DUBINS_EPS<T>()) < DUBINS_EPS<T>());
        return DubinsPathCeres::Path<T>(t, p, q, rho);
    }
    return DubinsPathCeres::Path<T>(); // Return default path if no valid path exists
}

template <typename T>
DubinsPathCeres::Path<T> dubinsRLR(T d, T alpha, T beta, T rho)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    T tmp = static_cast<T>(0.125) * (static_cast<T>(6) - d * d + static_cast<T>(2) * (ca * cb + sa * sb + d * (sa - sb)));
    if (ceres::abs(tmp) < static_cast<T>(1))
    {
        T p = twopi<T>() - ceres::acos(tmp);
        T theta = ceres::atan2(ca - cb, d - sa + sb);
        T t = mod2pi<T>(alpha - theta + static_cast<T>(0.5) * p);
        T q = mod2pi<T>(alpha - beta - t + p);
        assert(ceres::abs(static_cast<T>(2) * ceres::sin(alpha - t + p) - static_cast<T>(2) * ceres::sin(alpha - t) - d + sa - sb) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(ceres::abs(-static_cast<T>(2) * ceres::cos(alpha - t + p) + static_cast<T>(2) * ceres::cos(alpha - t) - ca + cb) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(mod2pi<T>(alpha - t + p - q - beta + static_cast<T>(0.5) * DUBINS_EPS<T>()) < DUBINS_EPS<T>());
        return DubinsPathCeres::Path<T>(t, p, q, rho);
    }
    return DubinsPathCeres::Path<T>(); // Return default path if no valid path exists
}

template <typename T>
DubinsPathCeres::Path<T> dubinsLRL(T d, T alpha, T beta, T rho)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    T tmp = static_cast<T>(0.125) * (static_cast<T>(6) - d * d + static_cast<T>(2) * (ca * cb + sa * sb - d * (sa - sb)));
    if (ceres::abs(tmp) < static_cast<T>(1))
    {
        T p = twopi<T>() - ceres::acos(tmp);
        T theta = ceres::atan2(-ca + cb, d + sa - sb);
        T t = mod2pi<T>(-alpha + theta + static_cast<T>(0.5) * p);
        T q = mod2pi<T>(beta - alpha - t + p);
        assert(ceres::abs(-static_cast<T>(2) * ceres::sin(alpha + t - p) + static_cast<T>(2) * ceres::sin(alpha + t) - d - sa + sb) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(ceres::abs(static_cast<T>(2) * ceres::cos(alpha + t - p) - static_cast<T>(2) * ceres::cos(alpha + t) + ca - cb) < static_cast<T>(2) * DUBINS_EPS<T>());
        assert(mod2pi<T>(alpha + t - p + q - beta + static_cast<T>(0.5) * DUBINS_EPS<T>()) < DUBINS_EPS<T>());
        return DubinsPathCeres::Path<T>(t, p, q, rho);
    }
    return DubinsPathCeres::Path<T>(); // Return default path if no valid path exists
}

// Templated longpath_thres_dist
template <typename T>
T longpath_thres_dist(T alpha, T beta)
{
    return ceres::abs(ceres::sin(alpha)) + ceres::abs(ceres::sin(beta)) +
           ceres::sqrt(static_cast<T>(4) - ceres::pow(ceres::cos(alpha) + ceres::cos(beta), static_cast<T>(2)));
}

// Templated is_longpath_case
template <typename T>
bool is_longpath_case(T d, T alpha, T beta)
{
    return (longpath_thres_dist(alpha, beta) - d) < static_cast<T>(0);
}

// Templated getDubinsClass
template <typename T>
DubinsClass getDubinsClass(const T alpha, const T beta)
{

    int row = 0, column = 0;

    if (T(0) <= alpha && alpha <= half_pi<T>()) {
        row = 1;
    }
    else if (half_pi<T>() < alpha && alpha <= pi<T>()) {
        row = 2;
    }
    else if (pi<T>() < alpha && alpha <= T(3) * half_pi<T>()) {
        row = 3;
    }
    else if (T(3) * half_pi<T>() < alpha && alpha <= twopi<T>()) {
        row = 4;
    }

    if (T(0) <= beta && beta <= half_pi<T>()) {
        column = 1;
    }
    else if (half_pi<T>() < beta && beta <= pi<T>()) {
        column = 2;
    }
    else if (pi<T>() < beta && beta <= T(3) * half_pi<T>()) {
        column = 3;
    }
    else if (T(3) * half_pi<T>() < beta &&
             beta <= T(2.0) * pi<T>()) {
        column = 4;
    }

    assert(row >= 1 && row <= 4 &&
           "alpha is not in the range of [0,2pi] in classifyPath(T alpha, T beta).");
    assert(column >= 1 && column <= 4 &&
           "beta is not in the range of [0,2pi] in classifyPath(T alpha, T beta).");
    assert((column - 1) + 4 * (row - 1) >= 0 && (column - 1) + 4 * (row - 1) <= 15 &&
           "class is not in range [0,15].");

    return static_cast<DubinsClass>((column - 1) + 4 * (row - 1));
}


// Templated helper functions
template <typename T>
inline T t_lsr(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = static_cast<T>(-2.0) + d * d + static_cast<T>(2.0) * (ca * cb + sa * sb + d * (sa + sb));
    const T p = ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));
    const T theta = ceres::atan2(-ca - cb, d + sa + sb) - ceres::atan2(static_cast<T>(-2.0), p);
    return mod2pi<T>(-alpha + theta);  // t
}

template <typename T>
inline T p_lsr(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = static_cast<T>(-2.0) + d * d + static_cast<T>(2.0) * (ca * cb + sa * sb + d * (sa + sb));
    return ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));  // p
}

template <typename T>
inline T q_lsr(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = static_cast<T>(-2.0) + d * d + static_cast<T>(2.0) * (ca * cb + sa * sb + d * (sa + sb));
    const T p = ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));
    const T theta = ceres::atan2(-ca - cb, d + sa + sb) - ceres::atan2(static_cast<T>(-2.0), p);
    return mod2pi<T>(-beta + theta);  // q
}

template <typename T>
inline T t_rsl(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = d * d - static_cast<T>(2.0) + static_cast<T>(2.0) * (ca * cb + sa * sb - d * (sa + sb));
    const T p = ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));
    const T theta = ceres::atan2(ca + cb, d - sa - sb) - ceres::atan2(static_cast<T>(2.0), p);
    return mod2pi<T>(alpha - theta);  // t
}

template <typename T>
inline T p_rsl(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = d * d - static_cast<T>(2.0) + static_cast<T>(2.0) * (ca * cb + sa * sb - d * (sa + sb));
    return ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));  // p
}

template <typename T>
inline T q_rsl(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = d * d - static_cast<T>(2.0) + static_cast<T>(2.0) * (ca * cb + sa * sb - d * (sa + sb));
    const T p = ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));
    const T theta = ceres::atan2(ca + cb, d - sa - sb) - ceres::atan2(static_cast<T>(2.0), p);
    return mod2pi<T>(beta - theta);  // q
}

template <typename T>
inline T t_rsr(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T theta = ceres::atan2(ca - cb, d - sa + sb);
    return mod2pi<T>(alpha - theta);  // t
}

template <typename T>
inline T p_rsr(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = static_cast<T>(2.0) + d * d - static_cast<T>(2.0) * (ca * cb + sa * sb - d * (sb - sa));
    return ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));  // p
}

template <typename T>
inline T q_rsr(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T theta = ceres::atan2(ca - cb, d - sa + sb);
    return mod2pi<T>(-beta + theta);  // q
}

template <typename T>
inline T t_lsl(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T theta = ceres::atan2(cb - ca, d + sa - sb);
    return mod2pi<T>(-alpha + theta);  // t
}

template <typename T>
inline T p_lsl(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T tmp = static_cast<T>(2.0) + d * d - static_cast<T>(2.0) * (ca * cb + sa * sb - d * (sa - sb));
    return ceres::sqrt(std::max(tmp, static_cast<T>(0.0)));  // p
}

template <typename T>
inline T q_lsl(T d, T alpha, T beta)
{
    T ca = ceres::cos(alpha), sa = ceres::sin(alpha), cb = ceres::cos(beta), sb = ceres::sin(beta);
    const T theta = ceres::atan2(cb - ca, d + sa - sb);
    return mod2pi<T>(beta - theta);  // q
}

// Templated s functions
template <typename T>
inline T s_12(T d, T alpha, T beta)
{
    return p_rsr<T>(d, alpha, beta) - p_lsr<T>(d, alpha, beta) - static_cast<T>(2.0) * (q_rsl<T>(d, alpha, beta) - T(M_PI));
}

template <typename T>
inline T s_13(T d, T alpha, T beta)
{  // t_rsr - pi
    return t_rsr<T>(d, alpha, beta) - T(M_PI);
}

template <typename T>
inline T s_14_1(T d, T alpha, T beta)
{
    return t_rsr<T>(d, alpha, beta) - T(M_PI);
}

template <typename T>
inline T s_21(T d, T alpha, T beta)
{
    return p_lsl<T>(d, alpha, beta) - p_rsl<T>(d, alpha, beta) - static_cast<T>(2.0) * (t_rsl<T>(d, alpha, beta) - T(M_PI));
}

template <typename T>
inline T s_22_1(T d, T alpha, T beta)
{
    return p_lsl<T>(d, alpha, beta) - p_rsl<T>(d, alpha, beta) - static_cast<T>(2.0) * (t_rsl<T>(d, alpha, beta) - T(M_PI));
}

template <typename T>
inline T s_22_2(T d, T alpha, T beta)
{
    return p_rsr<T>(d, alpha, beta) - p_rsl<T>(d, alpha, beta) - static_cast<T>(2.0) * (q_rsl<T>(d, alpha, beta) - T(M_PI));
}

template <typename T>
inline T s_24(T d, T alpha, T beta)
{
    return q_rsr<T>(d, alpha, beta) - T(M_PI);
}

template <typename T>
inline T s_31(T d, T alpha, T beta)
{
    return q_lsl<T>(d, alpha, beta) - T(M_PI);
}

template <typename T>
inline T s_33_1(T d, T alpha, T beta)
{
    return p_rsr<T>(d, alpha, beta) - p_lsr<T>(d, alpha, beta) - static_cast<T>(2.0) * (t_lsr<T>(d, alpha, beta) - T(M_PI));
}

template <typename T>
inline T s_33_2(T d, T alpha, T beta)
{
    return p_lsl<T>(d, alpha, beta) - p_lsr<T>(d, alpha, beta) - static_cast<T>(2.0) * (q_lsr<T>(d, alpha, beta) - T(M_PI));
}

template <typename T>
inline T s_34(T d, T alpha, T beta)
{
    return p_rsr<T>(d, alpha, beta) - p_lsr<T>(d, alpha, beta) - static_cast<T>(2.0) * (t_lsr<T>(d, alpha, beta) - T(M_PI));
}

template <typename T>
inline T s_41_1(T d, T alpha, T beta)
{
    return t_lsl<T>(d, alpha, beta) - T(M_PI);
}

template <typename T>
inline T s_41_2(T d, T alpha, T beta)
{
    return q_lsl<T>(d, alpha, beta) - T(M_PI);
}

template <typename T>
inline T s_42(T d, T alpha, T beta)
{
    return t_lsl<T>(d, alpha, beta) - T(M_PI);
}

template <typename T>
inline T s_43(T d, T alpha, T beta)
{
    return p_lsl<T>(d, alpha, beta) - p_lsr<T>(d, alpha, beta) - static_cast<T>(2.0) * (q_lsr<T>(d, alpha, beta) - T(M_PI));
}

// Templated dubins_classification
template <typename T>
DubinsPathCeres::Path<T> dubins_classification(T d, T alpha, T beta, T rho)
{
    using namespace fromOMPLCeres;
    if (d < DUBINS_EPS<T>() && ceres::abs(alpha - beta) < DUBINS_EPS<T>())
        return DubinsPathCeres::Path<T>(static_cast<T>(0), d, static_cast<T>(0), rho);

    // Dubins set classification scheme
    // Shkel, Andrei M., and Vladimir Lumelsky. "Classification of the Dubins set."
    //   Robotics and Autonomous Systems 34.4 (2001): 179-202.
    // Lim, Jaeyoung, et al. "Circling Back: Dubins set Classification Revisited."
    //   Workshop on Energy Efficient Aerial Robotic Systems, International Conference on Robotics and Automation 2023.
    //   2023.
    DubinsPathCeres::Path<T> path;
    DubinsClass dubins_class = getDubinsClass<T>(alpha, beta);
    //DubinsClass dubins_class = DubinsClass::A11;
    switch (dubins_class)
    {
    case DubinsClass::A11:
    {
        path = dubinsRSL<T>(d, alpha, beta, rho);
        break;
    }

    case DubinsClass::A12:
    {
        if (s_13<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            path = (s_12<T>(d, alpha, beta) < static_cast<T>(0.0)) ? dubinsRSR<T>(d, alpha, beta, rho) : dubinsRSL<T>(d, alpha, beta, rho);
        }
        else
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
            DubinsPathCeres::Path<T> tmp = dubinsRSL<T>(d, alpha, beta, rho);
            if (path.p + path.q > tmp.p + tmp.q) // Assuming length is p + q for comparison
            {
                path = tmp;
            }
        }
        break;
    }

    case DubinsClass::A13:
    {
        if (s_13<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            path = dubinsRSR<T>(d, alpha, beta, rho);
        }
        else
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
        }
        break;
    }
    case DubinsClass::A14:
    {
        if (s_14_1<T>(d, alpha, beta) > static_cast<T>(0.0))
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
        }
        else if (s_24<T>(d, alpha, beta) > static_cast<T>(0.0))
        {
            path = dubinsRSL<T>(d, alpha, beta, rho);
        }
        else
        {
            path = dubinsRSR<T>(d, alpha, beta, rho);
        }
        break;
    }

    case DubinsClass::A21:
    {
        if (s_31<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            if (s_21<T>(d, alpha, beta) < static_cast<T>(0.0))
            {
                path = dubinsLSL<T>(d, alpha, beta, rho);
            }
            else
            {
                path = dubinsRSL<T>(d, alpha, beta, rho);
            }
        }
        else
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
            DubinsPathCeres::Path<T> tmp = dubinsRSL<T>(d, alpha, beta, rho);
            if (path.p + path.q > tmp.p + tmp.q) // Assuming length is p + q for comparison
            {
                path = tmp;
            }
        }
        break;
    }
    case DubinsClass::A22:
    {
        if (alpha > beta)
        {
            path = (s_22_1<T>(d, alpha, beta) < static_cast<T>(0.0)) ? dubinsLSL<T>(d, alpha, beta, rho) : dubinsRSL<T>(d, alpha, beta, rho);
        }
        else
        {
            path = (s_22_2<T>(d, alpha, beta) < static_cast<T>(0.0)) ? dubinsRSR<T>(d, alpha, beta, rho) : dubinsRSL<T>(d, alpha, beta, rho);
        }
        break;
    }
    case DubinsClass::A23:
    {
        path = dubinsRSR<T>(d, alpha, beta, rho);
        break;
    }
    case DubinsClass::A24:
    {
        if (s_24<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            path = dubinsRSR<T>(d, alpha, beta, rho);
        }
        else
        {
            path = dubinsRSL<T>(d, alpha, beta, rho);
        }
        break;
    }
    case DubinsClass::A31:
    {
        if (s_31<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            path = dubinsLSL<T>(d, alpha, beta, rho);
        }
        else
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
        }
        break;
    }
    case DubinsClass::A32:
    {
        path = dubinsLSL<T>(d, alpha, beta, rho);
        break;
    }
    case DubinsClass::A33:
    {
        if (alpha < beta)
        {
            if (s_33_1<T>(d, alpha, beta) < static_cast<T>(0.0))
            {
                path = dubinsRSR<T>(d, alpha, beta, rho);
            }
            else
            {
                path = dubinsLSR<T>(d, alpha, beta, rho);
            }
        }
        else
        {
            if (s_33_2<T>(d, alpha, beta) < static_cast<T>(0.0))
            {
                path = dubinsLSL<T>(d, alpha, beta, rho);
            }
            else
            {
                path = dubinsLSR<T>(d, alpha, beta, rho);
            }
        }
        break;
    }
    case DubinsClass::A34:
    {
        if (s_24<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            if (s_34<T>(d, alpha, beta) < static_cast<T>(0.0))
            {
                path = dubinsRSR<T>(d, alpha, beta, rho);
            }
            else
            {
                path = dubinsLSR<T>(d, alpha, beta, rho);
            }
        }
        else
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
            DubinsPathCeres::Path<T> tmp = dubinsRSL<T>(d, alpha, beta, rho);
            if (path.p + path.q > tmp.p + tmp.q) // Assuming length is p + q for comparison
            {
                path = tmp;
            }
        }
        break;
    }
    case DubinsClass::A41:
    {
        if (s_41_1<T>(d, alpha, beta) > static_cast<T>(0.0))
        {
            path = dubinsRSL<T>(d, alpha, beta, rho);
        }
        else if (s_41_2<T>(d, alpha, beta) > static_cast<T>(0.0))
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
        }
        else
        {
            path = dubinsLSL<T>(d, alpha, beta, rho);
        }
        break;
    }
    case DubinsClass::A42:
    {
        if (s_42<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            path = dubinsLSL<T>(d, alpha, beta, rho);
        }
        else
        {
            path = dubinsRSL<T>(d, alpha, beta, rho);
        }
        break;
    }
    case DubinsClass::A43:
    {
        if (s_42<T>(d, alpha, beta) < static_cast<T>(0.0))
        {
            if (s_43<T>(d, alpha, beta) < static_cast<T>(0.0))
            {
                path = dubinsLSL<T>(d, alpha, beta, rho);
            }
            else
            {
                path = dubinsLSR<T>(d, alpha, beta, rho);
            }
        }
        else
        {
            path = dubinsLSR<T>(d, alpha, beta, rho);
            DubinsPathCeres::Path<T> tmp = dubinsRSL<T>(d, alpha, beta, rho);
            if (path.p + path.q > tmp.p + tmp.q) // Assuming length is p + q for comparison
            {
                path = tmp;
            }
        }
        break;
    }
    case DubinsClass::A44:
    {
        path = dubinsLSR<T>(d, alpha, beta, rho);
        break;
    }

    }
    return path;
}

// Templated dubins_exhaustive
template <typename T>
DubinsPathCeres::Path<T> dubins_exhaustive(T d, T alpha, T beta, T rho)
{
    if (d < DUBINS_EPS<T>() && ceres::abs(alpha - beta) < DUBINS_EPS<T>())
        return DubinsPathCeres::Path<T>(static_cast<T>(0), d, static_cast<T>(0), rho);

    DubinsPathCeres::Path<T> path = dubinsLSL<T>(d, alpha, beta, rho);
    DubinsPathCeres::Path<T> tmp = dubinsRSR<T>(d, alpha, beta, rho);
    T len, minLength = path.p + path.q; // Assuming length is p + q

    if ((len = tmp.p + tmp.q) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsRSL<T>(d, alpha, beta, rho);
    if ((len = tmp.p + tmp.q) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsLSR<T>(d, alpha, beta, rho);
    if ((len = tmp.p + tmp.q) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsRLR<T>(d, alpha, beta, rho);
    if ((len = tmp.p + tmp.q) < minLength)
    {
        minLength = len;
        path = tmp;
    }
    tmp = dubinsLRL<T>(d, alpha, beta, rho);
    if ((len = tmp.p + tmp.q) < minLength)
        path = tmp;
    return path;
}
}




// Example usage:
// To use with double
// auto path = fromOMPL::dubins_classification<double>(d, alpha, beta);

// To use with float
// auto path = fromOMPL::dubins_classification<float>(d, alpha, beta);



#endif // FROMOMPL_CERES_HPP
