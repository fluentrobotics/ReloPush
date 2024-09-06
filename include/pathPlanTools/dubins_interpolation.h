#ifndef DUBINS_INTERPOLATION_H
#define DUBINS_INTERPOLATION_H

#include<graphTools/edge_path_info.h>

// interpolate push paths (approaching path already in edgepath)
std::pair<std::shared_ptr<std::vector<State>>,PathInfoList> interpolate_dubins(graphTools::EdgePathInfo& edgePathInfo, float path_resolution=0.1, std::string vertex_name="")
{
    std::string vName = vertex_name;
    if(vName == "")
        vName = edgePathInfo.vertices.getSourceName();

    float turning_rad = edgePathInfo.path.get_turning_radius();
    // path info list
    PathInfoList plist = PathInfoList();

    // augment pre-relocation (relocation to make it LP) first
    // do not multi-process    
    std::vector<std::vector<State>> preReloPaths(0);
    for(auto& it : edgePathInfo.pre_relocations) //first: source second: target
    {
        // for each pre-relocation (usually one)
        // find dubins path (intiutively straight)
        auto dubins_pre = findDubins(it.preReloDubins.startState, it.preReloDubins.targetState, turning_rad);
        
        //interpolate
#pragma region interploation_prerelo
        std::vector<State> preReloStateVec(0);
        if(it.use_dubins) // dubins or manual path
        {
            auto l_pre = dubins_pre.lengthCost(); // unit cost * turning rad
            auto num_pts_pre = static_cast<size_t>(l_pre/path_resolution);
            preReloStateVec.resize(num_pts_pre);
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
        }
        else // use manual path instead
        {
            preReloStateVec = *it.manual_path;
        }

#pragma endregion

        // add to path info
        PathInfo p(vName,moveType::pre,it.preReloDubins.startState, it.preReloDubins.targetState,preReloStateVec,std::vector<bool>(preReloStateVec.size(),true));
        plist.push_back(p);
        // for each prerelocation (mostly one)
        auto path_poses = it.pathToNextPush->getPath(true); // approach path
        PathInfo p_app(vName,moveType::app,it.preReloDubins.startState, it.preReloDubins.targetState,path_poses,std::vector<bool>(path_poses.size(),true));
        plist.push_back(p_app);   
          
        for(auto& _p : path_poses) // todo: use better way to augment vector
        {
            preReloStateVec.push_back(_p);
        }

        preReloPaths.push_back(preReloStateVec);
      
    }

    //State startState = edgePathInfo.sourceState;
    State startState = edgePathInfo.finalPushState;
#pragma region interpolation
    // interpolate
    auto l = edgePathInfo.path.lengthCost(); // unit cost * turning rad
    auto num_pts = static_cast<size_t>(l/path_resolution);
    //size_t num_pts = static_cast<size_t>(partial_path_info.path.length()/0.4); //todo: get resolution as a param
    //if(preReloPaths.size()>0)
    //    startState = State(edgePathInfo.pre_relocations.back().preReloDubins.targetState.x,
    //                        edgePathInfo.pre_relocations.back().preReloDubins.targetState.y,
    //                        startState.yaw); // relocated object nominal pose

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
        jeeho_interpolate(dubinsStart, edgePathInfo.path.omplDubins, (double)np / (double)num_pts, interState, &dubinsSpace,
                        turning_rad);

        State tempState(interState->getX(), interState->getY(),interState->getYaw());

        //single_path.poses[np] = one_pose;
        //final_path.push_back(tempState);
        main_push_path[np] = tempState;
    }
#pragma endregion

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

    // add path info
    PathInfo p(vName,moveType::final,edgePathInfo.sourceState,edgePathInfo.targetState,main_push_path,std::vector<bool>(main_push_path.size(),true)); //vertex name of delivering object
    plist.push_back(p);


    return std::make_pair(std::make_shared<std::vector<State>>(out_path),plist);
}


#endif



