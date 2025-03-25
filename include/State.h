#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <tuple>
#include <memory>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <FromOMPL.h>

typedef boost::geometry::model::d2::point_xy<double> Point;


namespace ReloPush
{
    struct State
    {

        State(double x, double y, double yaw, int time = 0) : x(x), y(y), yaw(yaw), time(time)
        {
            // rot.resize(2, 2);
            // rot(0, 0) = cos(-this->yaw);
            // rot(0, 1) = -sin(-this->yaw);
            // rot(1, 0) = sin(-this->yaw);
            // rot(1, 1) = cos(-this->yaw);
        }

        State() = default;

        bool operator==(const State &s) const
        {
            //return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);

            if(abs(x - s.x) < 0.00000001 && abs(y - s.y) < 0.00000001)
                return true;

            return false;
        }

        bool isSamePose(const State &s) const
        {
            if(abs(x-s.x)<0.000001 && abs(y-s.y) < 0.000001 && abs(fromOMPL::mod2pi(yaw)-fromOMPL::mod2pi(s.yaw)) < 0.000001)
                return true;

            return false;
        }

        bool agentCollision(const State &other, float LF, float carWidth) const
        {
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

        State get_prePush(double push_angle, double pre_push_distance)
        {
            State outState = State(x,y,push_angle);

            // Calculate the new x and y coordinates
            outState.x -= pre_push_distance * cos(outState.yaw);
            outState.y -= pre_push_distance * sin(outState.yaw);

            // change angle range
            outState.yaw = fromOMPL::mod2pi(outState.yaw);

            return outState;
        }

        void print(bool add_line = true)
        {
            std::cout << "[" << x << ", " << y << ", " << yaw << "]";
            if(add_line)
                std::cout << "," << std::endl;
            else
                std::cout << ";";
        }

        double x;
        double y;
        double yaw;
        int time;

    private:
        boost::numeric::ublas::matrix<double> rot;
        Point corner1, corner2, corner3, corner4;
    };

    static inline float StateDistance(const State &s1, const State &s2)
    {
        float dx = s2.x - s1.x;
        float dy = s2.y - s1.y;
        return sqrtf(dx * dx + dy * dy);
    }

    // duplicate with path_length in reloPush_tools.h
    static inline float StatePathlength(std::vector<State> &path)
    {
        float totalLength = 0.0;

        for (size_t i = 1; i < path.size(); ++i)
        {
            float dx = path[i].x - path[i - 1].x;
            float dy = path[i].y - path[i - 1].y;
            totalLength += std::sqrt(dx * dx + dy * dy);
        }

        return totalLength;
    }

    typedef std::shared_ptr<State> StatePtr;
    typedef std::vector<State> StatePath;
    typedef std::shared_ptr<StatePath> StatePathPtr;
    typedef std::vector<StatePath> StatePathList;
    typedef std::shared_ptr<StatePathList> StatePathListPtr;
    typedef std::vector<StatePathPtr> StatePathPtrList;
}




#endif
