#ifndef POINT_H
#define POINT_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

struct GeometryPoint {
    double x, y;
};

using Corners = std::vector<GeometryPoint>;

inline std::pair<double, double> project(const Corners& corners, const GeometryPoint& axis) {
    double min_d = std::numeric_limits<double>::infinity();
    double max_d = -std::numeric_limits<double>::infinity();
    for (const auto& c : corners) {
        double dot = c.x * axis.x + c.y * axis.y;
        min_d = std::min(min_d, dot);
        max_d = std::max(max_d, dot);
    }
    return {min_d, max_d};
}

inline bool overlap(std::pair<double, double> p1, std::pair<double, double> p2) {
    return p1.second >= p2.first && p2.second >= p1.first;
}

inline std::vector<GeometryPoint> get_axes(const Corners& corners) {
    std::vector<GeometryPoint> axes;
    for (size_t i = 0; i < 4; ++i) {
        GeometryPoint p1 = corners[i];
        GeometryPoint p2 = corners[(i + 1) % 4];
        double ex = p2.x - p1.x;
        double ey = p2.y - p1.y;
        GeometryPoint normal = {-ey, ex};
        double norm = std::hypot(normal.x, normal.y);
        if (norm > 0) {
            normal.x /= norm;
            normal.y /= norm;
            axes.push_back(normal);
        }
    }
    return axes;
}

inline Corners get_corners(double x, double y, double yaw, double front, double rear, double width) {
    double cos = std::cos(yaw);
    double sin = std::sin(yaw);
    Corners corners = {
        {x - rear * cos - (width / 2) * sin, y - rear * sin + (width / 2) * cos},
        {x + front * cos - (width / 2) * sin, y + front * sin + (width / 2) * cos},
        {x + front * cos + (width / 2) * sin, y + front * sin - (width / 2) * cos},
        {x - rear * cos + (width / 2) * sin, y - rear * sin - (width / 2) * cos}
    };
    return corners;
}

inline bool rectangles_intersect(const Corners& corners1, const Corners& corners2) {
    auto axes1 = get_axes(corners1);
    auto axes2 = get_axes(corners2);
    std::vector<GeometryPoint> all_axes;
    all_axes.reserve(axes1.size() + axes2.size());
    all_axes.insert(all_axes.end(), axes1.begin(), axes1.end());
    all_axes.insert(all_axes.end(), axes2.begin(), axes2.end());
    for (const auto& axis : all_axes) {
        auto p1 = project(corners1, axis);
        auto p2 = project(corners2, axis);
        if (!overlap(p1, p2)) return false;
    }
    return true;
}

struct Pose : public GeometryPoint {
    double yaw = 0.0;

    Pose(){}

    Pose(double x_in, double y_in, double yaw_in) : yaw(yaw_in)
    {
        x = x_in;
        y = y_in;
    }
};

inline Pose offsetPose(Pose& pose_in, double offset_dist)
{
    double op_x = pose_in.x + offset_dist * std::cos(pose_in.yaw);
    double op_y = pose_in.y + offset_dist * std::sin(pose_in.yaw);

    return Pose(op_x,op_y,pose_in.yaw);
}


#endif // POINT_H
