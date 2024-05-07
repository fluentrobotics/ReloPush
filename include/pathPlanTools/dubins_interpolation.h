#ifndef DUBINS_INTERPOLATION_H
#define DUBINS_INTERPOLATION_H

#include<graphTools/edge_path_info.h>


std::shared_ptr<std::vector<State>> interpolate_dubins(graphTools::EdgePathInfo& pathInfo, float turning_rad, float path_resolution=0.1)
{
    // interpolate
    auto l = pathInfo.path.length();
    auto num_pts = static_cast<size_t>(l/path_resolution);
    //size_t num_pts = static_cast<size_t>(partial_path_info.path.length()/0.4); //todo: get resolution as a param

    State startState = pathInfo.sourceState;

    ompl::base::DubinsStateSpace dubinsSpace(turning_rad);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(startState.x, startState.y);
    dubinsStart->setYaw(startState.yaw);
    OmplState *interState = (OmplState *)dubinsSpace.allocState();

    std::vector<State> out_vec(num_pts);

    // interpolate dubins path
    // Interpolate dubins path to check for collision on grid map
    //nav_msgs::Path single_path;
    //single_path.poses.resize(num_pts);
    for (size_t np=0; np<num_pts; np++)
    {            
        //auto start = std::chrono::steady_clock::now();
        jeeho_interpolate(dubinsStart, pathInfo.path, (double)np / (double)num_pts, interState, &dubinsSpace,
                        turning_rad);

        State tempState(interState->getX(), interState->getY(),interState->getYaw());

        //single_path.poses[np] = one_pose;
        //final_path.push_back(tempState);
        out_vec[np] = tempState;
    }

    return std::make_shared<std::vector<State>>(out_vec);
}


#endif



