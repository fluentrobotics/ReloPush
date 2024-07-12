#ifndef DUBINS_INTERPOLATION_H
#define DUBINS_INTERPOLATION_H

#include<graphTools/edge_path_info.h>


std::shared_ptr<std::vector<State>> interpolate_dubins(graphTools::EdgePathInfo& pathInfo, float path_resolution=0.1)
{
    float turning_rad = pathInfo.path.get_turning_radius();
    // augment pre-relocation (relocation to make it LP) first
    // do not multi-process
    
    std::vector<std::vector<State>> preReloPaths(0);
    for(auto& it : pathInfo.pre_relocations) //first: source second: target
    {
        // for each pre-relocation (usually one)
        // find dubins path (intiutively straight)
        auto dubins_pre = findDubins(it.preReloDubins.startState, it.preReloDubins.targetState, turning_rad);
        auto l_pre = dubins_pre.lengthCost(); // unit cost * turning rad
        auto num_pts_pre = static_cast<size_t>(l_pre/path_resolution);

        //interpolate
        std::vector<State> preReloStateVec(num_pts_pre);

        State startState = it.preReloDubins.startState;

        ompl::base::DubinsStateSpace dubinsSpace(turning_rad);
        OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
        dubinsStart->setXY(startState.x, startState.y);
        dubinsStart->setYaw(startState.yaw);
        OmplState *interState = (OmplState *)dubinsSpace.allocState();

        for (size_t np=0; np<num_pts_pre; np++)
        {            
            //auto start = std::chrono::steady_clock::now();
            jeeho_interpolate(dubinsStart, dubins_pre.omplDubins, (double)np / (double)num_pts_pre, interState, &dubinsSpace,
                            turning_rad);

            State tempState(interState->getX(), interState->getY(),interState->getYaw());

            //single_path.poses[np] = one_pose;
            //final_path.push_back(tempState);
            preReloStateVec[np] = tempState;
        }


        // for each prerelocation (mostly one)
        auto path_poses = it.pathToNextPush->getPath(true);

        for(auto& p : path_poses) // todo: use better way to augment vector
        {
            preReloStateVec.push_back(p);
        }
        

        preReloPaths.push_back(preReloStateVec);
    }

    // interpolate
    auto l = pathInfo.path.lengthCost(); // unit cost * turning rad
    auto num_pts = static_cast<size_t>(l/path_resolution);
    //size_t num_pts = static_cast<size_t>(partial_path_info.path.length()/0.4); //todo: get resolution as a param

    State startState = pathInfo.sourceState;
    if(preReloPaths.size()>0)
        startState = State(pathInfo.pre_relocations.back().preReloDubins.targetState.x,
                            pathInfo.pre_relocations.back().preReloDubins.targetState.y,
                            startState.yaw); // relocated object nominal pose

    ompl::base::DubinsStateSpace dubinsSpace(turning_rad);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(startState.x, startState.y);
    dubinsStart->setYaw(startState.yaw);
    OmplState *interState = (OmplState *)dubinsSpace.allocState();

    std::vector<State> main_push_path(num_pts);

    // interpolate dubins path
    // Interpolate dubins path to check for collision on grid map
    //nav_msgs::Path single_path;
    //single_path.poses.resize(num_pts);
    for (size_t np=0; np<num_pts; np++)
    {            
        //auto start = std::chrono::steady_clock::now();
        jeeho_interpolate(dubinsStart, pathInfo.path.omplDubins, (double)np / (double)num_pts, interState, &dubinsSpace,
                        turning_rad);

        State tempState(interState->getX(), interState->getY(),interState->getYaw());

        //single_path.poses[np] = one_pose;
        //final_path.push_back(tempState);
        main_push_path[np] = tempState;
    }

    //combine with pre-relocations
    std::vector<State> out_path(0);
    for(auto& it : preReloPaths)
    {
        // relocation push
        for(auto& prerelo_wpt : it)
            out_path.push_back(prerelo_wpt);
    }
    for(auto& it : main_push_path)
    {
        out_path.push_back(it);
    }


    return std::make_shared<std::vector<State>>(out_path);
}


#endif



