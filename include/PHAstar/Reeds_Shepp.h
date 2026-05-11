/************************************************************************************************************************
 * Reeds-Shepp implementation converted from PythonRobotics
 * https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py
 *
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
************************************************************************************************************************/


#ifndef REEDS_SHEPP_H
#define REEDS_SHEPP_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <tuple>
#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include <functional>
#include <unordered_map>

#include <PHAstar/Utils.h>

// Reeds-Shepp Path Planning Implementation
namespace ReedShepp
{
    struct Path {
        std::vector<double> lengths;
        std::string ctypes;
        double L = 0.0;
        std::vector<double> x, y, yaw;
        std::vector<int> directions;
        std::vector<double> steers;
        bool is_reflected = false;  // Added to track if path is from reflected space
    };


    inline std::pair<double, double> polar(double x, double y) {
        double r = std::hypot(x, y);
        double theta = std::atan2(y, x);
        return {r, theta};
    }

    inline std::vector<Path> set_path(std::vector<Path> paths, const std::vector<double>& lengths, const std::string& ctypes, double step_size, bool reflected = false) {
        Path path;
        path.ctypes = ctypes;
        path.lengths = lengths;
        path.L = 0.0;
        for (double len : lengths) path.L += std::abs(len);
        path.is_reflected = reflected;  // Set the reflected flag

        bool exists = false;
        for (const auto& p : paths) {
            if (p.ctypes == path.ctypes && std::abs(p.L - path.L) <= step_size) {
                exists = true;
                break;
            }
        }
        if (!exists && path.L > step_size) {
            paths.push_back(path);
        }
        return paths;
    }

    constexpr double eps = 1e-6;

    inline std::tuple<bool, std::vector<double>, std::string> left_straight_left(double x, double y, double phi) {
        auto [u, t] = polar(x - std::sin(phi), y - 1.0 + std::cos(phi));
        if (t >= -eps && t <= M_PI) {  // Relaxed
            double v = mod2pi(phi - t);
            if (v >= -eps && v <= M_PI) {  // Relaxed
                return {true, {t, u, v}, "LSL"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_straight_right(double x, double y, double phi) {
        auto [u1, t1] = polar(x + std::sin(phi), y - 1.0 - std::cos(phi));
        u1 *= u1;
        if (u1 >= 4.0) {
            double u = std::sqrt(u1 - 4.0);
            double theta = std::atan2(2.0, u);
            double t = mod2pi(t1 + theta);
            double v = mod2pi(t - phi);
            if (t >= -eps && v >= -eps) {  // Relaxed
                return {true, {t, u, v}, "LSR"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_x_right_x_left(double x, double y, double phi) {
        double zeta = x - std::sin(phi);
        double eeta = y - 1 + std::cos(phi);
        auto [u1, theta] = polar(zeta, eeta);
        if (u1 <= 4.0) {
            double A = std::acos(0.25 * u1);
            double t = mod2pi(A + theta + M_PI / 2);
            double u = mod2pi(M_PI - 2 * A);
            double v = mod2pi(phi - t - u);
            if (t >= -eps && u >= -eps && v >= -eps) {  // Relaxed, add for u
                return {true, {t, -u, v}, "LRL"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_x_right_left(double x, double y, double phi) {
        double zeta = x - std::sin(phi);
        double eeta = y - 1 + std::cos(phi);
        auto [u1, theta] = polar(zeta, eeta);
        if (u1 <= 4.0) {
            double A = std::acos(0.25 * u1);
            double t = mod2pi(A + theta + M_PI / 2);
            double u = mod2pi(M_PI - 2 * A);
            double v = mod2pi(-phi + t + u);
            if (t >= -eps && u >= -eps && v >= -eps) {  // Relaxed
                return {true, {t, -u, -v}, "LRL"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_right_x_left(double x, double y, double phi) {
        double zeta = x - std::sin(phi);
        double eeta = y - 1 + std::cos(phi);
        auto [u1, theta] = polar(zeta, eeta);
        if (u1 <= 4.0) {
            double u = std::acos(1 - u1 * u1 * 0.125);
            double A = std::asin(2 * std::sin(u) / u1);
            double t = mod2pi(-A + theta + M_PI / 2);
            double v = mod2pi(t - u - phi);
            if (t >= -eps && u >= -eps && v >= -eps) {  // Relaxed
                return {true, {t, u, -v}, "LRL"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_right_x_left_right(double x, double y, double phi) {
        double zeta = x + std::sin(phi);
        double eeta = y - 1 - std::cos(phi);
        auto [u1, theta] = polar(zeta, eeta);
        if (u1 <= 2.0) {
            double A = std::acos((u1 + 2) * 0.25);
            double t = mod2pi(theta + A + M_PI / 2);
            double u = mod2pi(A);
            double v = mod2pi(phi - t + 2 * u);
            if (t >= -eps && u >= -eps && v >= -eps) {  // Relaxed
                return {true, {t, u, -u, -v}, "LRLR"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_x_right_left_x_right(double x, double y, double phi) {
        double zeta = x + std::sin(phi);
        double eeta = y - 1 - std::cos(phi);
        auto [u1, theta] = polar(zeta, eeta);
        double u2 = (20 - u1 * u1) / 16;
        if (0 <= u2 && u2 <= 1) {
            double u = std::acos(u2);
            double A = std::asin(2 * std::sin(u) / u1);
            double t = mod2pi(theta + A + M_PI / 2);
            double v = mod2pi(phi - t + u);
            if (t >= -eps && u >= -eps && v >= -eps) {  // Relaxed
                return {true, {t, -u, u, -v}, "LRLR"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_x_right90_straight_left(double x, double y, double phi) {
        auto [u, t] = polar(x - std::sin(phi), y - 1 + std::cos(phi));
        if (t >= 0.0) {
            double v = mod2pi(phi - t + M_PI / 2);
            if (v >= 0.0) {
                return {true, {t, -M_PI / 2, u, v}, "LrSL"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_straight_right90_x_left(double x, double y, double phi) {
        auto [u, t] = polar(x + std::sin(phi), y - 1 - std::cos(phi));
        if (t >= 0.0) {
            double v = mod2pi(phi - t - M_PI / 2);
            if (v >= 0.0) {
                return {true, {t, u, -M_PI / 2, v}, "LSrL"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_x_right90_straight_right(double x, double y, double phi) {
        auto [u1, t1] = polar(x + std::sin(phi), y - 1 - std::cos(phi));
        u1 = u1 * u1;
        if (u1 >= 4.0) {
            double u = std::sqrt(u1 - 4.0);
            double theta = std::atan2(2.0, u);
            double t = mod2pi(t1 + theta);
            double v = mod2pi(t + M_PI / 2 - phi);
            if (t >= 0.0 && v >= 0.0) {
                return {true, {t, -M_PI / 2, u, v}, "LrSR"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_straight_left90_x_right(double x, double y, double phi) {
        auto [u1, t1] = polar(x - std::sin(phi), y - 1 + std::cos(phi));
        u1 = u1 * u1;
        if (u1 >= 4.0) {
            double u = std::sqrt(u1 - 4.0);
            double theta = std::atan2(2.0, u);
            double t = mod2pi(t1 + theta);
            double v = mod2pi(t - M_PI / 2 - phi);
            if (t >= 0.0 && v >= 0.0) {
                return {true, {t, u, M_PI / 2, v}, "LSlR"};
            }
        }
        return {false, {}, ""};
    }

    inline std::tuple<bool, std::vector<double>, std::string> left_x_right90_straight_left90_x_right(double x, double y, double phi) {
        auto [u1, t1] = polar(x + std::sin(phi), y - 1 - std::cos(phi));
        u1 = u1 * u1;
        if (u1 >= 4.0) {
            double u = std::sqrt(u1 - 4.0);
            double theta = std::atan2(2.0, u);
            double t = mod2pi(t1 + theta);
            double v = mod2pi(t + M_PI / 2 - phi);
            if (t >= 0.0 && v >= 0.0) {
                return {true, {t, -M_PI / 2, u, M_PI / 2, v}, "LrSlr"};
            }
        }
        return {false, {}, ""};
    }

    inline std::vector<double> timeflip(const std::vector<double>& lengths) {
        std::vector<double> flipped;
        for (double d : lengths) flipped.push_back(-d);
        return flipped;
    }

    inline std::string reflect(const std::string& dirs) {
        std::string reflected;
        for (char d : dirs) {
            if (d == 'L') reflected += 'R';
            else if (d == 'R') reflected += 'L';
            else reflected += 'S';
        }
        return reflected;
    }

    inline std::vector<Path> generate_path(double sx, double sy, double syaw, double gx, double gy, double gyaw, double maxc, double step_size) {
        double dx = gx - sx;
        double dy = gy - sy;
        double dth = gyaw - syaw;
        double c = std::cos(syaw);
        double s = std::sin(syaw);
        double x_ = (c * dx + s * dy) * maxc;
        double y_ = (-s * dx + c * dy) * maxc;
        step_size *= maxc;

        std::vector<Path> paths;
        std::vector<std::tuple<bool, std::vector<double>, std::string> (*)(double, double, double)> path_funcs = {
          left_straight_left, left_straight_right,
          left_x_right_x_left, left_x_right_left, left_right_x_left,
          left_right_x_left_right, left_x_right_left_x_right,
          left_x_right90_straight_left, left_straight_right90_x_left,
          left_x_right90_straight_right, left_straight_left90_x_right,
          left_x_right90_straight_left90_x_right
        };

        for (auto func : path_funcs) {
            auto [flag, travels, dirs] = func(x_, y_, dth);
            if (flag) paths = set_path(paths, travels, dirs, step_size);

            std::tie(flag, travels, dirs) = func(-x_, y_, -dth);
            if (flag) paths = set_path(paths, timeflip(travels), dirs, step_size);

            std::tie(flag, travels, dirs) = func(x_, -y_, -dth);
            if (flag) paths = set_path(paths, travels, reflect(dirs), step_size);

            std::tie(flag, travels, dirs) = func(-x_, -y_, dth);
            if (flag) paths = set_path(paths, timeflip(travels), reflect(dirs), step_size);
        }

        return paths;
    }

    inline std::tuple<double, double, double, int, double> interpolate(double dist, double length, char mode, double maxc, double ox, double oy, double oyaw, double wb) {
        double x, y, yaw, steer = 0.0;
        int direction;
        if (mode == 'S') {
            direction = (length > 0.0) ? 1 : -1;
            x = ox + dist / maxc * std::cos(oyaw);
            y = oy + dist / maxc * std::sin(oyaw);
            yaw = oyaw;
            steer = 0.0;
        } else {
            direction = (length > 0.0) ? 1 : -1;
            double ldx = std::sin(dist) / maxc;
            double ldy = 0.0;
            double sign_mode = (mode == 'L') ? 1.0 : -1.0;
            steer = sign_mode * std::atan(maxc * wb);
            if (mode == 'L') {
                ldy = (1.0 - std::cos(dist)) / maxc;
                yaw = oyaw + dist;
            } else if (mode == 'R') {
                ldy = (1.0 - std::cos(dist)) / -maxc;
                yaw = oyaw - dist;
            }
            double gdx = std::cos(-oyaw) * ldx + std::sin(-oyaw) * ldy;
            double gdy = -std::sin(-oyaw) * ldx + std::cos(-oyaw) * ldy;
            x = ox + gdx;
            y = oy + gdy;
        }
        return {x, y, yaw, direction, steer};
    }

    inline std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<int>, std::vector<double>> generate_local_course(const std::vector<double>& lengths, const std::string& modes, double maxc, double step_size, double wb) {
        std::vector<double> xs, ys, yaws, steers;
        std::vector<int> directions;
        double ox = 0.0, oy = 0.0, oyaw = 0.0;

        for (size_t i = 0; i < lengths.size(); ++i) {
            double len = lengths[i];
            char mode = modes[i];
            double d_dist = (len >= 0.0) ? step_size : -step_size;
            double curr = 0.0;
            while (std::abs(curr) < std::abs(len)) {
                auto [ix, iy, iyaw, dir, is] = interpolate(curr, len, mode, maxc, ox, oy, oyaw, wb);
                xs.push_back(ix);
                ys.push_back(iy);
                yaws.push_back(iyaw);
                directions.push_back(dir);
                steers.push_back(is);
                curr += d_dist;
            }
            auto [ix, iy, iyaw, dir, is] = interpolate(len, len, mode, maxc, ox, oy, oyaw, wb);
            xs.push_back(ix);
            ys.push_back(iy);
            yaws.push_back(iyaw);
            directions.push_back(dir);
            steers.push_back(is);
            ox = xs.back();
            oy = ys.back();
            oyaw = yaws.back();
        }
        return {xs, ys, yaws, directions, steers};
    }

    inline std::vector<Path> calc_paths(double sx, double sy, double syaw, double gx, double gy, double gyaw, double maxc, double step_size, double wb) {
        auto paths = generate_path(sx, sy, syaw, gx, gy, gyaw, maxc, step_size);
        std::vector<Path> valid_paths;
        for (auto& path : paths) {
            auto [xs, ys, yaws, dirs, steers] = generate_local_course(path.lengths, path.ctypes, maxc, step_size * maxc, wb);


            path.x.resize(xs.size());
            path.y.resize(ys.size());
            path.yaw.resize(yaws.size());
            path.directions = dirs;
            path.steers = steers;
            for (size_t i = 0; i < xs.size(); ++i) {
                path.x[i] = std::cos(-syaw) * xs[i] + std::sin(-syaw) * ys[i] + sx;
                path.y[i] = -std::sin(-syaw) * xs[i] + std::cos(-syaw) * ys[i] + sy;
                path.yaw[i] = pi_2_pi(yaws[i] + syaw);
            }
            for (auto& len : path.lengths) len /= maxc;
            path.L /= maxc;

            double last_x = path.x.back();
            double last_y = path.y.back();
            double last_yaw = mod2pi(path.yaw.back());
            double goal_yaw = mod2pi(gyaw);
            if (std::hypot(last_x - gx, last_y - gy) > 1e-5 || std::fabs(pi_2_pi(last_yaw - goal_yaw)) > 1e-5) {
                continue;  // Skip invalid due to FP
            }
            valid_paths.push_back(path);
        }
        return valid_paths;
    }

    inline std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::string, std::vector<double>, std::vector<double>, std::vector<int>> reeds_shepp_path_planning(double sx, double sy, double syaw, double gx, double gy, double gyaw, double maxc, double step_size, double wb) {
        auto paths = calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size, wb);
        if (paths.empty()) return {{}, {}, {}, "", {}, {}, {}};
        auto min_it = std::min_element(paths.begin(), paths.end(), [](const Path& a, const Path& b){ return std::abs(a.L) < std::abs(b.L); });
        const Path& b_path = *min_it;
        return {b_path.x, b_path.y, b_path.yaw, b_path.ctypes, b_path.lengths, b_path.steers, b_path.directions};
    }
}

#endif // REEDS_SHEPP_H
