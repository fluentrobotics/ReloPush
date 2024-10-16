#ifndef STATE_H
#define STATE_H

#include<iostream>
#include <tuple>
#include <memory>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

typedef boost::geometry::model::d2::point_xy<double> Point;



struct State
{
    State(double x, double y, double yaw, int time = 0) : x(x), y(y), yaw(yaw), time(time)
    {
        //rot.resize(2, 2);
        //rot(0, 0) = cos(-this->yaw);
        //rot(0, 1) = -sin(-this->yaw);
        //rot(1, 0) = sin(-this->yaw);
        //rot(1, 1) = cos(-this->yaw);
    }

    State() = default;

    bool operator==(const State &s) const
    {
        return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
    }

    bool agentCollision(const State& other, float LF, float carWidth) const {
        if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
            pow(2 * LF, 2) + pow(carWidth, 2))
        return true;
        return false;    
    }

    State(const State &) = default;
    State(State &&) = default;
    State &operator=(const State &) = default;
    State &operator=(State &&) = default;

    friend std::ostream &operator<<(std::ostream &os, const State &s)
    {
        return os << "(" << s.x << "," << s.y << "," << s.yaw << ")";
    }

    double x;
    double y;
    double yaw;
    int time;

    private:
    boost::numeric::ublas::matrix<double> rot;
    Point corner1, corner2, corner3, corner4;
};

static inline float StateDistance(const State& s1, const State& s2)
{
    float dx = s2.x - s1.x;
    float dy = s2.y - s1.y;
    return sqrtf(dx * dx + dy * dy);
}

// duplicate with path_length in reloPush_tools.h
static inline float StatePathlength(std::vector<State>& path)
{
    float totalLength = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        float dx = path[i].x - path[i-1].x;
        float dy = path[i].y - path[i-1].y;
        totalLength += std::sqrt(dx * dx + dy * dy);
    }

    return totalLength;
}


typedef std::shared_ptr<State> StatePtr;
typedef std::vector<State> StatePath;
typedef std::shared_ptr<StatePath> StatePathPtr;
typedef std::vector<StatePath> StatePathList;
typedef std::shared_ptr<StatePathList> StatePathListPtr;


#endif