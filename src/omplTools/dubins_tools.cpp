#include <omplTools/dubins_tools.h>

#define M_PI 3.14159265358979323846 /* pi */

//namespace ob = ompl::base;
//namespace og = ompl::geometric;
//namespace po = boost::program_options;

//typedef ompl::base::SE2StateSpace::StateType OmplState;


void jeeho_interpolate(const OmplState *from, const ompl::base::DubinsStateSpace::DubinsPath &path, double t,
                       OmplState *state, ompl::base::DubinsStateSpace* space, double turning_radius)
{
    OmplState *s = space->allocState()->as<OmplState>();
    double seg = t * path.length(), phi=0, v=0;

    s->setXY(0,0);
    s->setYaw(from->getYaw());
    std::cout << "S x: " << s->getX() << " " << s->getY() << " " << s->getYaw() << std::endl;
    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[i])
            {
                case ompl::base::DubinsStateSpace::DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi + v) - sin(phi), s->getY() - cos(phi + v) + cos(phi));
                    s->setYaw(phi + v);
                    break;
                case ompl::base::DubinsStateSpace::DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi - v) + sin(phi), s->getY() + cos(phi - v) - cos(phi));
                    s->setYaw(phi - v);
                    break;
                case ompl::base::DubinsStateSpace::DUBINS_STRAIGHT:
                    s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                    break;
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[2 - i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[2 - i])
            {
                case ompl::base::DubinsStateSpace::DUBINS_LEFT:  // DUBINS_LEFT
                    s->setXY(s->getX() + sin(phi - v) - sin(phi), s->getY() - cos(phi - v) + cos(phi));
                    s->setYaw(phi - v);
                    break;
                case ompl::base::DubinsStateSpace::DUBINS_RIGHT:  // DUBINS_RIGHT
                    s->setXY(s->getX() - sin(phi + v) + sin(phi), s->getY() + cos(phi + v) - cos(phi));
                    s->setYaw(phi + v);
                    break;
                case ompl::base::DubinsStateSpace::DUBINS_STRAIGHT:  // DUBINS_STRAIGHT
                    s->setXY(s->getX() - v * cos(phi), s->getY() - v * sin(phi));
                    break;
            }
        }
    }

    std::cout << "---" << s->getX() << " " << s->getY() << std::endl;

    state->setX(s->getX() * turning_radius + from->getX());
    state->setY(s->getY() * turning_radius + from->getY());
    space->getSubspace(1)->enforceBounds(s->as<OmplState>(1));
    state->setYaw(s->getYaw());
    space->freeState(s);
}


ompl::base::DubinsStateSpace::DubinsPath findDubins(State &start, State &goal, double turning_radius)
{
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    OmplState *dubinsEnd = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(start.x, start.y);
    dubinsStart->setYaw(-start.yaw);
    dubinsEnd->setXY(goal.x, goal.y);
    dubinsEnd->setYaw(-goal.yaw);
    ompl::base::DubinsStateSpace::DubinsPath dubinsPath = dubinsSpace.dubins(dubinsStart, dubinsEnd);
    dubinsStart->setXY(start.x, start.y);
    dubinsStart->setYaw(-start.yaw);

    for (auto pathidx = 0; pathidx < 3; pathidx++)
    {
        switch (dubinsPath.type_[pathidx])
        {
            case 0:  // DUBINS_LEFT
                std::cout << "Left" << std::endl;
                break;
            case 1:  // DUBINS_STRAIGHT
                std::cout << "Straight" << std::endl;
                break;
            case 2:  // DUBINS_RIGHT
                std::cout << "Right" << std::endl;
                break;
            default:
                std::cout << "\033[1m\033[31m"
                          << "Warning: Receive unknown DubinsPath type"
                          << "\033[0m\n";
                break;
        }
        std::cout << fabs(dubinsPath.length_[pathidx]) << std::endl;
    }

    OmplState *interState = (OmplState *)dubinsSpace.allocState();
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = 100;

    for (size_t n=0; n<num_pts; n++)
    {
        //auto start = std::chrono::steady_clock::now();
        jeeho_interpolate(dubinsStart, dubinsPath, (double)n / (double)num_pts, interState, &dubinsSpace,
                          turning_radius);
        //auto end = std::chrono::steady_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        //std::cout << "Elapsed time: " << duration << " usec" << std::endl;

        std::cout << interState->getX() << " " << interState->getY() << " " << interState->getYaw() << std::endl;
    }

    return dubinsPath;
}

// Function to transform a point from the global frame to the robot's frame
Eigen::Vector2d worldToRobot(double x, double y, double theta, double robot_x, double robot_y) {
    auto dx_world = x - robot_x;
    auto dy_world = y - robot_y;

    Eigen::Matrix2d rotationMatrix;
    rotationMatrix << cos(theta), sin(theta),
                       -sin(theta), cos(theta);

    Eigen::Vector2d point;
    point << dx_world, dy_world;
    return rotationMatrix * point;
}

std::pair<pathType,dubinsPath> is_good_path(State& s1, State& s2, float turning_rad)
{
    double x1 = s1.x, y1 = s1.y, th1 = s1.yaw;
    double x2 = s2.x, y2 = s2.y, th2 = s2.yaw;
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / turning_rad, th = atan2(dy, dx);
    double alpha = fromOMPL::mod2pi(th1 - th), beta = fromOMPL::mod2pi(th2 - th);

    if (d < fromOMPL::DUBINS_EPS && fabs(alpha - beta) < fromOMPL::DUBINS_EPS)
        return std::make_pair<pathType,dubinsPath>(pathType::none,{ompl::base::DubinsStateSpace::dubinsPathType[0], 0, 0, 0}); //zero dubins path

    alpha = fromOMPL::mod2pi(alpha);
    beta = fromOMPL::mod2pi(beta);
    bool is_long = fromOMPL::is_longpath_case(d, alpha, beta); 
    
    if(!is_long) // short-path
    {
        //dubinsPath dubinsSet = fromOMPL::dubins_classification(d, alpha, beta); //path
        dubinsPath dubinsSet = fromOMPL::dubins_exhaustive(d, alpha, beta);
        return std::make_pair<pathType,dubinsPath>(pathType::SP,{dubinsSet.type_, dubinsSet.length_[0], dubinsSet.length_[1], dubinsSet.length_[2]});
    }

    //if longpath case
    //find dubins set
    dubinsPath dubinsSet = fromOMPL::dubins_classification(d, alpha, beta); //path

    double dubins_distance = dubinsSet.length() * turning_rad;
    //find dx and dy i.r.t. robot
    auto dx_dy_robot = worldToRobot(s2.x,s2.y,s1.yaw,s1.x,s1.y);
    //std::cout << "x: " << dx_dy_robot[0] << " y: " << dx_dy_robot[1] << std::endl;
    auto dx_dy_sum = dx_dy_robot[0] + dx_dy_robot[1];
    // tie-break
    dx_dy_sum *= 1.01;

    if(dubins_distance > dx_dy_sum) // large-turn long-path
        return std::make_pair<pathType,dubinsPath>(pathType::largeLP,{dubinsSet.type_, dubinsSet.length_[0], dubinsSet.length_[1], dubinsSet.length_[2]});
    else // small-turn long-path
        return std::make_pair<pathType,dubinsPath>(pathType::smallLP,{dubinsSet.type_, dubinsSet.length_[0], dubinsSet.length_[1], dubinsSet.length_[2]});
}
