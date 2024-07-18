#include <omplTools/dubins_tools.h>

#define M_PI 3.14159265358979323846 /* pi */

//namespace ob = ompl::base;
//namespace og = ompl::geometric;
//namespace po = boost::program_options;

//typedef ompl::base::SE2StateSpace::StateType OmplState;

double sumUpToIndex(const double* arr, size_t length, unsigned int i) {
    // Check if the index is within bounds
    if (i >= length) {
        throw std::out_of_range("Index is out of range");
    }

    double sum = 0.0;
    for (size_t j = 0; j <= i; ++j) {
        sum += arr[j];
    }
    return sum;
}


void jeeho_interpolate(const OmplState *from, const ompl::base::DubinsStateSpace::DubinsPath &path, double t,
                       OmplState *state, ompl::base::DubinsStateSpace* space, double turning_radius)
{
    OmplState *s = space->allocState()->as<OmplState>();
    double seg = t * path.length(), phi=0, v=0;

    s->setXY(0,0);
    s->setYaw(from->getYaw());
    //std::cout << "S x: " << s->getX() << " " << s->getY() << " " << s->getYaw() << std::endl;
    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i]);
            //v = std::min(seg, sumUpToIndex(path.length_,3,i)*turning_radius);
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

    //std::cout << "---" << s->getX() << " " << s->getY() << std::endl;

    state->setX(s->getX() * turning_radius + from->getX());
    state->setY(s->getY() * turning_radius + from->getY());
    //state->setX(s->getX() + from->getX());
    //state->setY(s->getY() + from->getY());
    space->getSubspace(1)->enforceBounds(s->as<OmplState>(1));
    state->setYaw(s->getYaw());
    space->freeState(s);
}


reloDubinsPath findDubins(State &start, State &goal, double turning_radius, bool print_type)
{
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    OmplState *dubinsEnd = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(start.x, start.y);
    //dubinsStart->setYaw(-start.yaw);
    dubinsStart->setYaw(jeeho::convertEulerRange_to_2pi(start.yaw));
    dubinsEnd->setXY(goal.x, goal.y);
    //dubinsEnd->setYaw(-goal.yaw);
    dubinsEnd->setYaw(jeeho::convertEulerRange_to_2pi(goal.yaw));

    //for debug
    auto xx = dubinsEnd->getX();
    auto yy = dubinsEnd->getY();

    ompl::base::DubinsStateSpace::DubinsPath dPath = dubinsSpace.dubins(dubinsStart, dubinsEnd);

    // inherited class
    reloDubinsPath dubinsPath(dPath,turning_radius);

    dubinsStart->setXY(start.x, start.y);
    dubinsStart->setYaw(-start.yaw);

    if(print_type)
    {
        for (auto pathidx = 0; pathidx < 3; pathidx++)
        {
            switch (dubinsPath.omplDubins.type_[pathidx])
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
            std::cout << fabs(dubinsPath.omplDubins.length_[pathidx]) << std::endl;
        }
    }

/*
    OmplState *interState = (OmplState *)dubinsSpace.allocState();
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = 100;

    for (size_t n=0; n<num_pts; n++)
    {
        //auto start = std::chrono::steady_clock::now();
        jeeho_interpolate(dubinsStart, dubinsPath.omplDubins, (double)n / (double)num_pts, interState, &dubinsSpace,
                          turning_radius);
        //auto end = std::chrono::steady_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        //std::cout << "Elapsed time: " << duration << " usec" << std::endl;

        std::cout << interState->getX() << " " << interState->getY() << " " << interState->getYaw() << std::endl;
    }
    */

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

void find_alpha_beta_ompl(State& s1, State& s2, double& alpha_out, double& beta_out)
{
    double x1 = s1.x, y1 = s1.y, th1 = s1.yaw;
    double x2 = s2.x, y2 = s2.y, th2 = s2.yaw;
    double dx = x2 - x1, dy = y2 - y1, th = atan2(dy, dx);
    alpha_out = fromOMPL::mod2pi(th1 - th), beta_out = fromOMPL::mod2pi(th2 - th);
}

// returns euclidean distance threshold (not normalized by turning radius)
float get_longpath_d_thres(State& s1, State& s2, float turning_rad)
{
    //double alpha, beta;
    //find_alpha_beta_ompl(s1,s2,alpha,beta);

    return static_cast<float>(fromOMPL::longpath_thres_dist(s1.yaw,s2.yaw))*turning_rad;
}

std::pair<pathType,reloDubinsPath> is_good_path(State& s1, State& s2, float turning_rad, bool use_pre_push_pose)
{
    //double x1 = s1.x, y1 = s1.y, th1 = s1.yaw;
    //double x2 = s2.x, y2 = s2.y, th2 = s2.yaw;
    //double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / turning_rad, th = atan2(dy, dx);
    //double alpha = fromOMPL::mod2pi(th1 - th), beta = fromOMPL::mod2pi(th2 - th);
    
    //OMPL uses slightly different alpha and beta, which lead to different results
    double a=fromOMPL::mod2pi(s1.yaw), b=fromOMPL::mod2pi(s2.yaw); 
    //find_alpha_beta(s1,s2,alpha,beta);
    double dx = s2.x - s1.x, dy = s2.y - s1.y, d = sqrt(dx * dx + dy * dy) / turning_rad;

    if(use_pre_push_pose)
    {}

    // these are as good as same poses
    if (d < fromOMPL::DUBINS_EPS && fabs(a - b) < fromOMPL::DUBINS_EPS)
        return std::make_pair<pathType,reloDubinsPath>(pathType::none,{ompl::base::DubinsStateSpace::dubinsPathType[0], 0, 0, 0}); //zero dubins path

    // alpha = fromOMPL::mod2pi(alpha); // duplicate
    // beta = fromOMPL::mod2pi(beta); // duplicate
    bool is_long = fromOMPL::is_longpath_case(d, a, b); 
    
    double alpha, beta;
    find_alpha_beta_ompl(s1,s2,alpha,beta); //todo: investigate if OMPL's alpha and beta is needed

    if(!is_long) // short-path
    {
        //dubinsPath dubinsSet = fromOMPL::dubins_classification(d, alpha, beta); //path
        dubinsPath dubinsSet = fromOMPL::dubins_exhaustive(d, alpha, beta);
        return std::make_pair<pathType,reloDubinsPath>(pathType::SP,{dubinsSet.type_, dubinsSet.length_[0], dubinsSet.length_[1], dubinsSet.length_[2], turning_rad});
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
        return std::make_pair<pathType,reloDubinsPath>(pathType::largeLP,{dubinsSet.type_, dubinsSet.length_[0], dubinsSet.length_[1], dubinsSet.length_[2], turning_rad});
    else // small-turn long-path
        return std::make_pair<pathType,reloDubinsPath>(pathType::smallLP,{dubinsSet.type_, dubinsSet.length_[0], dubinsSet.length_[1], dubinsSet.length_[2], turning_rad});
}
