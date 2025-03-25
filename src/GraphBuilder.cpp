#include <GraphBuilder.hpp>
#include "DubinsTools.h"
#include <PathPlanningTools.h>
#include <boost/graph/graphviz.hpp>
#include <fstream>
#include <cmath>     // std::sqrt
#include <iostream>  // std::cout

// -----------------------------------------------------------------
// Add a single vertex
// -----------------------------------------------------------------
Vertex addVertexToGraph(
    Graph &g,
    VertexType type,
    const std::string &name,
    int orientationIdx,
    double nominalOri,
    double x,
    double y,
    int numberOfSides)
{
    Vertex v = boost::add_vertex(g);
    g[v].type = type;
    g[v].name = name;
    g[v].orientationIndex = orientationIdx;
    g[v].nominalOrientation = nominalOri;
    g[v].x = x;
    g[v].y = y;
    g[v].numberOfSides = numberOfSides;

    return v;
}

// -----------------------------------------------------------------
// Feasibility check
// -----------------------------------------------------------------
bool canConnect(const VertexData &from, const VertexData &to)
{
    // Example: check distance < some threshold
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    const double THRESHOLD = 100.0;  // e.g. 100 units
    return (dist < THRESHOLD);
}

bool canConnectNormal(const VertexData &from, const VertexData &to)
{
    // Put your "normalMode" feasibility logic here.
    // Example: distance < threshold, no collision, etc.
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    // Some condition for normalMode
    const double NORMAL_THRESHOLD = 100.0;
    return (dist < NORMAL_THRESHOLD);
}

bool canConnectPreRelocation(const VertexData &from, const VertexData &to)
{
    // Put your "preRelocation" logic here.
    // Maybe it's more lenient or a different constraint.
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    // Some condition for preRelocation
    // e.g., bigger threshold or some alternative check
    const double PRE_RELOC_THRESHOLD = 150.0;
    return (dist < PRE_RELOC_THRESHOLD);
}


ReloPush::State find_pre_push(ReloPush::State& goalState, float distance)
{
    ReloPush::State outState(goalState);

    // Calculate the new x and y coordinates
    outState.x -= distance * cos(goalState.yaw);
    outState.y -= distance * sin(goalState.yaw);

    // change angle range
    outState.yaw = fromOMPL::mod2pi(outState.yaw);

    return outState;
}


// Check Dubins Validity
StatePathValidity check_dubins_validity(reloDubinsPath& dubins_in, PlanningContext& ctx)
{
    // For each path points
    // Check boundary
    // Check collision

    auto l = dubins_in.lengthCost(); // unit cost * turning rad
    auto num_pts = static_cast<size_t>(l/ctx.parameters.map_resolution);

    ompl::base::DubinsStateSpace dubinsSpace(ctx.parameters.turning_rad_pair.push);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(dubins_in.startState.x, dubins_in.startState.y);
    dubinsStart->setYaw(dubins_in.startState.yaw);


    OmplState *interState = (OmplState *)dubinsSpace.allocState();


    std::vector<ReloPush::State> main_push_path(num_pts);

    bool failed_flag = false;
    StateValidity out_validity = StateValidity::valid;

    // interpolate dubins path
    // Interpolate dubins path to check for collision on grid map
    //nav_msgs::Path single_path;
    //single_path.poses.resize(num_pts);
    if(num_pts>0){
        for (size_t np=0; np<num_pts; np++)
        {
            //auto start = std::chrono::steady_clock::now();
            jeeho_interpolate(dubinsStart, dubins_in.omplDubins, (double)np / (double)num_pts, interState, &dubinsSpace,
                              ctx.parameters.turning_rad_pair.push);

            ReloPush::State tempState(interState->getX(), interState->getY(),interState->getYaw());
            main_push_path[np] = tempState; // failed State at the last

            // check boundary
            auto tempValid = ctx.env_push.stateValid(tempState, ctx.parameters.car_width, ctx.parameters.obs_rad,ctx.parameters.LF_push, ctx.parameters.LB); // obs as LF
            if(!tempValid)
            {
                // reason for failure
                out_validity = tempValid.get_validity();
                // raise failed flag
                failed_flag = true;

                break;
            }
        }
    }
    else{
        //std::cout << "Path too short to interpolate" << std::endl;
        main_push_path.resize(1);
        main_push_path[0] = dubins_in.targetState;
    } // path is too short there is nothing to interpolate

    StatePathValidity out_pair(std::make_shared<ReloPush::StatePath>(main_push_path), out_validity);

    return out_pair;
}

// For last ReloPush planning
/*
PathPlanResultPtr check_approach_validity(ReloPush::State preRelocation, ObjectInfo movingObject,
                            int landingOrientationIndex, int finalOrientationIndex, double angleChange,
                             PlanningContext& ctx)
{

    // object to move
    ObjectInfo object = movingObject;
    // apply angle change
    object.applyRotation(angleChange);

    // remove object from initial pose
    ctx.env.remove_obs(movingObject.getNominalPose());

    // add to obstacles
    ctx.env.add_obs(ReloPush::State(preRelocation.x,preRelocation.y, object.getOrientation(landingOrientationIndex)));

    // departing pose
    auto from_center_pose = ReloPush::State(preRelocation.x,preRelocation.y, object.getOrientation(landingOrientationIndex));
    // pre-push
    auto from_pre_push = find_pre_push(from_center_pose, ctx.parameters.PrePush_dist);

    // final landing pose
    //auto to_center_pose = ReloPush::State(preRelocation.x,preRelocation.y, object.getOrientation(finalOrientationIndex));
    // pre-push
    auto to_pre_push = find_pre_push(preRelocation, ctx.parameters.PrePush_dist);

    // plan hybrid astar
    auto res = planHybridAstar(from_pre_push, to_pre_push, ctx, true);

    // remove obs
    ctx.env.remove_obs(ReloPush::State(preRelocation.x,preRelocation.y, object.getOrientation(landingOrientationIndex)));

    // restore object
    ctx.env.add_obs(movingObject.getNominalPose());

    // return result
    return res;
}
*/

// For last ReloPush planning
PathPlanResultPtr check_approach_validity(ReloPush::State relocated_robot, ReloPush::State final_prepush, ReloPush::State preRelocation,
                                          ObjectInfo movingObject,
                                          int landingOrientationIndex, int finalOrientationIndex, double angleChange,
                                          PlanningContext& ctx)
{

    // object to move
    ObjectInfo object = movingObject;
    // obstacle temporary removal
    //auto temp_obs = movingObject.getNominalPose();
    // remove object from initial pose
    //ctx.env.remove_obs(temp_obs);
    // apply angle change
    object.applyRotation(angleChange);

    // add to obstacles
    ctx.addObs(ReloPush::State(preRelocation.x,preRelocation.y, object.getOrientation(landingOrientationIndex)));

    // plan hybrid astar
    auto res = planHybridAstar(relocated_robot, final_prepush, ctx, true);

    // remove obs
    ctx.removeObs(ReloPush::State(preRelocation.x,preRelocation.y, object.getOrientation(landingOrientationIndex)));

    // restore object
    //ctx.env.add_obs(temp_obs);

    // return result
    return res;
}

// For optimization-based planning
PathPlanResultPtr check_approach_validity(ReloPush::State preRelocation, ReloPush::State approachingPose,
                                          ObjectInfo movingObject, int landingOrientationIndex,
                                          int finalOrientationIndex, double angleChange, PlanningContext& ctx)
{
    if(preRelocation.isSamePose(approachingPose))
    {
        // start and goal are identical. return empty path
        PathPlanResult res_empty;
        res_empty.start_pose = preRelocation;
        res_empty.goal_pose = approachingPose;
        res_empty.cost = 0;
        res_empty.success = true;
        res_empty.validity = PlanValidity::success;
        return std::make_shared<PathPlanResult>(res_empty);
    }

    // object to move
    ObjectInfo object = movingObject;
    // apply angle change
    object.applyRotation(angleChange);

    // remove object from initial pose
    //ctx.env.remove_obs(movingObject.getNominalPose());

    auto obj_prerelo = ReloPush::revert_pre_push(preRelocation,ctx.parameters.PrePush_dist);
    // add to obstacles
    ctx.addObs(obj_prerelo);

    // departing pose
    //auto from_center_pose = ReloPush::State(preRelocation.x,preRelocation.y, object.getOrientation(landingOrientationIndex));
    // pre-push
    //auto from_pre_push = find_pre_push(from_center_pose, ctx.parameters.PrePush_dist);

    // final landing pose
    //auto to_center_pose = ReloPush::State(obj_prerelo.x,obj_prerelo.y, object.getOrientation(finalOrientationIndex)+angleChange);
    // pre-push
    //auto to_pre_push = find_pre_push(to_center_pose, ctx.parameters.PrePush_dist);

    // plan hybrid astar
    auto res = planHybridAstar(preRelocation, approachingPose, ctx, true); // start pose is already the car pose

    // remove obs
    ctx.removeObs(obj_prerelo);

    // restore object
    //ctx.env.add_obs(movingObject.getNominalPose());

    // return result
    return res;
}


//
// Normal Edge Connection (No Prerelocation)
//

StateValidity addEdgeNormalMode(
    Graph &g,
    Vertex v1,
    Vertex v2,
    PlanningContext& ctx
    )
{
    // 0) Extract vertex data
    const auto &data1 = g[v1];
    const auto &data2 = g[v2];

    // for debug only
    bool deb = false;
    if(data1.name == "box1" && data2.name == "goal2" && data1.orientationIndex==0 && data2.orientationIndex == 0)
        deb = true;

    // 1) Temporarily remove start from obstacle. If target is also an obstacle, remove it, too.
    std::vector<ReloPush::State> took_out(0);
    took_out.push_back(ReloPush::State(data1.x,data1.y,data1.nominalOrientation));
    ctx.removeObs(ReloPush::State(data1.x,data1.y,data1.nominalOrientation));
    if(data2.type==VertexType::OBJECT_VERTEX)
    {
        took_out.push_back(ReloPush::State(data2.x,data2.y,data2.nominalOrientation));
        ctx.removeObs(ReloPush::State(data2.x,data2.y,data2.nominalOrientation));
    }


    // Create start/goal states with (x, y, theta)
    ReloPush::State start(data1.x, data1.y, data1.getActualOrientation());
    ReloPush::State goal(data2.x, data2.y, data2.getActualOrientation());

    // check if goal is valid
    auto gv = ctx.env_push.stateValid(goal);
    if(gv.get_validity()!=StateValidity::valid)
        return gv.get_validity();

    //use prepush dist
    ReloPush::State start_prepush = find_pre_push(start,ctx.parameters.PrePush_dist);
    ReloPush::State goal_prepush = find_pre_push(goal,ctx.parameters.PrePush_dist);

    // 2) Call your Dubins planner.
    auto dubinsResult = PlanDubins(start_prepush, goal_prepush, ctx);

    // 3) Check if the path is valid (within boundary & collision-free)
    auto isPathValid = check_dubins_validity(dubinsResult.second, ctx);

    // put the obstacles back
    for(auto it : took_out)
        ctx.addObs(it);


    // 4) If all waypoints are valid => add the edge to the graph
    //    We'll store the path + its length in EdgeData
    if(isPathValid)
    {
        Edge e;
        bool inserted;
        boost::tie(e, inserted) = boost::add_edge(v1, v2, g);
        if (inserted)
        {
            g[e].weight = dubinsResult.second.lengthCost();           // path length from planner
            g[e].mode   = ConnectionMode::NORMAL_MODE;
            g[e].paths = {std::make_shared<EdgePath>(EdgePath(true,dubinsResult.second))};    // store entire path for reference
            g[e].srcVertexData = g[v1];
            g[e].sinkVertexData = g[v2];
        }
        return StateValidity::valid;
    }
    else
    {
        return isPathValid.path_validity; //reason for failure
    }
}

StateValidity addEdgePrerelocation(
    Graph &g,
    Vertex v1,
    Vertex v2,
    PlanningContext &ctx, StateValidity& reason_in)
{
    // 1) Info about the start/goal
    const auto &data1 = g[v1];
    const auto &data2 = g[v2];

    // for debug
    bool deb = false;
    if(data1.name == "box1" && data2.name == "goal2" && data1.orientationIndex == 0 && data2.orientationIndex==0)
        deb = true;

    // for debug
    //int obs_before = ctx.env.get_obs().size();

    ReloPush::State startPose(data1.x, data1.y, data1.getActualOrientation());
    ReloPush::State goalPose(data2.x, data2.y, data2.getActualOrientation());


    ReloPush::State goalPose_prepush = find_pre_push(goalPose, ctx.parameters.PrePush_dist);

    int nSides = data1.numberOfSides;
    //int startOriIndex = data1.orientationIndex; // the orientation that failed
    //double mapRes = 0.1; // or ctx.params.mapResolution, etc.

    // retrieve object info
    ObjectInfo movingObject = ctx.mo_list[data1.name];
    auto final_push_index = data1.orientationIndex;

    // Temporarily remove the start and goal from the obstacles list
    ctx.removeObs(startPose);
    if(data2.type==VertexType::OBJECT_VERTEX)
        ctx.removeObs(goalPose);

    // check goal validity
    auto gv = ctx.env_push.stateValid(goalPose);
    if(!gv)
    {
        ctx.addObs(startPose);
        if(data2.type==VertexType::OBJECT_VERTEX)
            ctx.addObs(goalPose);
        return gv.get_validity();
    }

    // set goal on map for costmap
    //ctx.env.changeGoal(goalPose);

    // 2) We'll attempt to relocate the start position in multiple directions
    //    derived from the "other" orientation indices of the object.
    //    e.g., if startOriIndex=0, we skip 0, try i=1..nSides-1
    //    or you can interpret "the axis" differently.
    //
    // For each orientation i != startOriIndex:
    //   directionAngle = data1.nominalOrientation + (2*pi / nSides) * i
    //   then we shift the original x,y along directionAngle by increments

    // Because user wants to keep the same "heading" but shift position,
    // we do *not* change 'startPose.yaw'. We only change 'x,y' along each axis.

    double bestCost = std::numeric_limits<double>::infinity();
    bool foundAny = false;
    ReloPush::State bestRelocated; // store best relocation found
    ReloPush::State bestRelocated_robot;
    ReloPush::State bestPreRelocation_object;
    ReloPush::State bestFinalPrepush;
    ReloPush::State bestStart_prepush;
    int bestOrientationIndex = -1;
    reloDubinsPath final_push_path;
    double bestPreRelo_orientation;

    double landingAngleChange = 0; // no change in landing orientation for this prerelocation

    // For each orientation axis
    for (int i = 0; i < nSides; ++i)
    {
        //if (i == startOriIndex)
        //    continue; // skip the original orientation used in normalMode

        double sideAngle = data1.nominalOrientation + (2.0*M_PI / nSides) * i;
        sideAngle = fromOMPL::mod2pi(sideAngle);
        // i is the orientation index


        // We'll do a simple loop for some fixed # of steps (e.g. up to distance 5?)
        // or until we find a feasible relocation
        double maxShiftDist = 5.0;  // you decide
        int maxSteps = static_cast<int>(maxShiftDist / ctx.parameters.map_resolution);

        for (int step = 1; step <= maxSteps; step++)
        {
            double shiftDist = step * ctx.parameters.map_resolution;
            //double xNew = startPose.x + shiftDist * std::cos(sideAngle);
            //double yNew = startPose.y + shiftDist * std::sin(sideAngle);
            // robot-centered path
            ReloPush::State preStartPose = ReloPush::State(startPose.x,startPose.y,sideAngle); // prepush for this prerelocation
            ReloPush::State startPose_prepush = find_pre_push(preStartPose, ctx.parameters.PrePush_dist);
            double xNew = startPose_prepush.x + shiftDist * std::cos(sideAngle);
            double yNew = startPose_prepush.y + shiftDist * std::sin(sideAngle);

            // Construct a new start
            ReloPush::State relocated_robot(xNew, yNew, sideAngle); //landing at prerelocation
            auto relocated_object = ReloPush::revert_pre_push(relocated_robot,ctx.parameters.PrePush_dist); // landing object at prerelocation

            // 2a) Quick boundary/collision checks if desired:
            if (!ctx.env_push.stateValid(relocated_object))
                break; // no reason to keep going further in this direction

            if (!ctx.env_push.stateValid(relocated_robot)) //todo: might by okay to skip this?
                break;

            ReloPush::State PreRelocation_object = ReloPush::State(relocated_object.x,relocated_object.y,startPose.yaw);

            // check if final appraoch is valid
            auto final_prepush = find_pre_push(PreRelocation_object, ctx.parameters.PrePush_dist);
            if(!ctx.env_push.stateValid(final_prepush))
                continue; // there may be other valid ones further toward this direction

            // 2b) Now check if is_longpath_case(...) says it's "good."
            if (!is_longpath_case(final_prepush, goalPose_prepush, ctx.parameters.turning_rad_pair.push))
            {
                // not a good candidate, try next step
                continue;
            }

            // 2c) If it's "good," compute total cost:
            //   relocation cost = distance from (start.x, start.y) to (xNew, yNew)
            double relocationDist = shiftDist;  // if we interpret shiftDist as Eucl. distance
            // Then plan a Dubins path from 'relocated' to 'goalPose'
            //   e.g. auto dubinsRes = PlanDubins(relocated, goalPose, ctx);

            auto dubinsRes = PlanDubins(final_prepush, goalPose_prepush, ctx);
            // check validity, etc.
            if (dubinsRes.first == pathType::SP) // todo: duplicate path planning
            {
                // dubins planner failed, skip
                continue;
            }

            // check if out-of-boundary
            if(isDubinsValid(dubinsRes.second,ctx)!=StateValidity::valid)
                continue;

            double dubinsCost = dubinsRes.second.lengthCost();
            double totalCost = relocationDist + dubinsCost;

            // 2d) If totalCost < bestCost, update best
            if (totalCost < bestCost)
            {
                bestCost = totalCost;
                bestRelocated = relocated_object; //todo: Do we need both object and robot?
                bestRelocated_robot = relocated_robot;
                bestPreRelo_orientation = movingObject.getNominalPose().yaw; // no change in orientation in this method
                bestPreRelocation_object = PreRelocation_object;
                bestStart_prepush = startPose_prepush;
                bestFinalPrepush = final_prepush;
                foundAny = true;
                bestOrientationIndex = i;
                final_push_path = dubinsRes.second; // todo: break?
            }

            // If you want to break as soon as you find the *first* feasible:
            // you can break here. If you want the best among all possible,
            // keep looping.

        }
    }

    StateValidity out_validity = StateValidity::out_of_boundary;

    // for debug
    //int obs_mid = ctx.env.get_obs().size();
    std::unordered_set<ReloPush::State> obs_mid1, obs_mid2;

    if (!foundAny)
    {
        // we never found a valid pre-relocation
        out_validity = StateValidity::out_of_boundary;
    }
    else
    {
        // for debug
        //obs_mid1 = ctx.env.get_obs();
        // check if approach to final push is feasible
        auto planApproach = check_approach_validity(bestRelocated_robot, bestFinalPrepush, bestPreRelocation_object, movingObject,
                                                    bestOrientationIndex, final_push_index, landingAngleChange, ctx);

        //obs_mid2 = ctx.env.get_obs();

        //bool env_debug = false;
        //if(obs_mid1.size() != obs_mid2.size())
        //    env_debug = true;

        if(planApproach->validity == PlanValidity::success)
        {
            // 3) We found some best relocation => create an edge in the graph
            Edge e;
            bool inserted;
            boost::tie(e, inserted) = boost::add_edge(v1, v2, g);

            if (inserted)
            {
                g[e].weight = bestCost; // all cost included
                g[e].mode   = ConnectionMode::PRE_RELOCATION;
                g[e].preRelo.used        = true;
                g[e].preRelo.xRelocated_robot  = bestRelocated.x;
                g[e].preRelo.yRelocated_robot  = bestRelocated.y;
                g[e].preRelo.yawReloacted_robot = bestPreRelo_orientation; // no orientation change for this method
                g[e].preRelo.xRelocated_object = bestPreRelocation_object.x;
                g[e].preRelo.yRelocated_object = bestPreRelocation_object.y;
                g[e].preRelo.yawRelocated_object = bestPreRelocation_object.yaw;
                g[e].preRelo.extraCost   = std::hypot(bestRelocated.x - startPose.x,
                                                    bestRelocated.y - startPose.y);                
                g[e].preRelo.relocatingIndex = bestOrientationIndex;
                g[e].preRelo.reason = reason_in;
                g[e].srcVertexData = g[v1];
                g[e].sinkVertexData = g[v2];

                // pre-relocation path
                double preRelo_orientation = movingObject.getOrientation(bestOrientationIndex);
                //ReloPush::State preReloPose_from(bestStart_prepush.x, startPose_prepush.y, preRelo_orientation);
                //ReloPush::State preReloPose_arrival(bestRelocated.x, bestRelocated.y, preRelo_orientation);
                auto preReloDubins = findDubins(bestStart_prepush, bestRelocated_robot, ctx.parameters.turning_rad_pair.push);
                EdgePath preReloPath(true, preReloDubins);

                // approach path
                EdgePath appPath(false, planApproach->getPathPtr(true));

                // final push path
                EdgePath finalPushPath(true, final_push_path);

                g[e].paths = {std::make_shared<EdgePath>(preReloPath), std::make_shared<EdgePath>(appPath), std::make_shared<EdgePath>(finalPushPath)};

                out_validity = StateValidity::valid;
            }
        }

        else
        {
            // failed
            out_validity = StateValidity::no_approach;
        }
    }
    //int obs_mid_last = ctx.env.get_obs().size();
    // restore start and goal as obstacles
    ctx.addObs(startPose);
    if(data2.type==VertexType::OBJECT_VERTEX)
        ctx.addObs(goalPose);

    //int obs_after = ctx.env.get_obs().size();

    return out_validity;
}



// add edge using optimiztion
StateValidity addEdgePrerelocation_Optimization(
    Graph &g,
    Vertex v1,
    Vertex v2,
    PlanningContext &ctx,
    StateValidity &reason_in)
{
    // 1) Gather start/goal info
    const auto &data1 = g[v1];
    const auto &data2 = g[v2];

    // for debug
    bool deb = false;
    if(data1.name=="box1" && data2.name=="goal1" && data1.orientationIndex==3 && data2.orientationIndex == 3)
        deb = true;

    // for debug
    //auto obs_before = ctx.env.get_obs();

    ReloPush::State startPose(data1.x, data1.y, data1.getActualOrientation()); // object
    ReloPush::State goalPose(data2.x, data2.y, data2.getActualOrientation()); // goal

    // Temporarily remove the start and goal from the obstacles list
    ctx.removeObs(startPose);
    if(data2.type==VertexType::OBJECT_VERTEX)
        ctx.removeObs(goalPose);

    // check goal validity

    auto gv = ctx.env_push.stateValid(goalPose);
    if(!gv)
    {
        ctx.addObs(startPose);
        if(data2.type==VertexType::OBJECT_VERTEX)
            ctx.addObs(goalPose);
        return gv.get_validity();
    }


    auto goalPose_prepush = find_pre_push(goalPose, ctx.parameters.PrePush_dist);

    ObjectInfo movingObject = ctx.mo_list[data1.name];
    auto final_push_index = data1.orientationIndex;

    int nSides = data1.numberOfSides;
    int startOriIndex = data1.orientationIndex; // orientation that failed normal mode
    double R = ctx.parameters.turning_rad_pair.push;

    double bestCost = std::numeric_limits<double>::infinity();
    bool foundAny = false;

    // We'll keep track of the best relocation result
    ReloPush::OptResult bestOpt(0.0, 0.0, 0.0, bestCost,0);
    double bestPreRelo_orientation = 0;
    int bestOrientationIndex = -1;
    reloDubinsPath bestDubins_prerelo, bestDubins_final;
    ReloPush::State bestPreRelo_object;
    double bestExtraCost = 0;

    // 2) For each orientation axis
    for (int i = 0; i < nSides; ++i)
    {
        // Optionally skip the orientation used by normal mode, if desired:
        // if (i == startOriIndex) continue;

        // compute the angle for orientation i
        double sideAngle = data1.nominalOrientation + (2.0 * M_PI / nSides) * i;
        auto pushingPose = ReloPush::State(startPose.x,startPose.y,sideAngle);

        auto startPose_prepush = find_pre_push(pushingPose, ctx.parameters.PrePush_dist);

        // 2a) Try all initial guesses from ctx.sampledPositions
        /*
        for (const auto &initPos : ctx.sampledPositions)
        {
            double x_init_guess = initPos.x;
            double y_init_guess = initPos.y;

            // Call your function:
            // (x_i, y_i, th_i, x2, y2, th2, sideAngle, R, x_init_guess, y_init_guess)
            OptResult optRes = FindPreRelocationOptimization(
                startPose.x,
                startPose.y,
                startPose.yaw,
                goalPose.x,
                goalPose.y,
                goalPose.yaw,
                sideAngle,
                R,
                x_init_guess,
                y_init_guess,
                ctx
                );

            double relocationX   = optRes.x;
            double relocationY   = optRes.y;
            double relocationYaw = optRes.landing_yaw;
            double costOpt       = optRes.cost; // cost from the solver
            double delta_yaw    = optRes.change_in_yaw;

            // If the cost is infinite or >= bestCost, skip
            if (costOpt >= bestCost)
                continue;

            // 2b) (Optional) boundary or collision checks:
            // if (!ctx.env.inBoundary(relocationX, relocationY)) continue;
            // if (someCollisionCheck(relocationX, relocationY)) continue;

            // 2c) If your cost function doesn't already include final path cost,
            //     plan a path from (relocationX,relocationY,relocationYaw) to goalPose.
            // double dubinsDist = ...
            // double totalCost = costOpt + dubinsDist;
            double totalCost = costOpt;

            // 2d) Keep the best
            if (totalCost < bestCost)
            {
                bestCost = totalCost;
                bestOpt  = OptResult(relocationX, relocationY, relocationYaw, costOpt, delta_yaw);
                bestOrientationIndex = i;

                bestDubins_prerelo = findDubins(ReloPush::State(startPose.x,startPose.y,sideAngle), ReloPush::State(relocationX,relocationY,sideAngle + bestOpt.change_in_yaw), ctx.parameters.turning_rad_pair.push);
                bestDubins_final = findDubins(ReloPush::State(relocationX,relocationY,relocationYaw), goalPose, ctx.parameters.turning_rad_pair.push);

                if(bestCost < 50) //todo: handle nan
                    foundAny = true;
            }
        } // end for sampledPositions
        */


        /*
        auto init_guess_xy = ReloPush::find_init_guess_intersection(goalPose.x,goalPose.y,goalPose.yaw,
                                                                    startPose_prepush.x,startPose_prepush.y,sideAngle,
                                                                    ctx.parameters.turning_rad_pair.push,sideAngle-startPose.yaw);


        // check NaN (parallel directions)
        if (std::isnan(init_guess_xy.first) || std::isnan(init_guess_xy.second))
        {
            // handle exception (ignore this option)
            continue;
        }

        double xc, yc;
        worldToLocal<double>(init_guess_xy.first, init_guess_xy.second, startPose_prepush.x, startPose_prepush.y, sideAngle, &xc, &yc);

        // skip if it is not in the forwad direction
        if(xc <0)
            continue;

        auto orientation_length = computeLocalOrientation<double>(xc, yc, ctx.parameters.turning_rad_pair.push);
        double th1pc = orientation_length.th1pc; // todo: is it same as th2 + (th_ip - th_i)?

        // use prepush for initial guess (result is robot pose)
        auto init_guess = ReloPush::State(init_guess_xy.first, init_guess_xy.second, sideAngle + th1pc);
        //auto init_guess_prepush = find_pre_push(init_guess,ctx.parameters.PrePush_dist);
        */

        auto init_guess = ReloPush::FindInitialGuess(Eigen::Vector3d(startPose_prepush.x,startPose_prepush.y,sideAngle),Eigen::Vector3d(goalPose.x,goalPose.y,goalPose.yaw),
                                                     ctx.parameters.PrePush_dist,ctx.parameters.turning_rad_pair.push,startPose.yaw);


        //double x_init_guess = init_guess_xy.first;
        //double y_init_guess = init_guess_xy.second;
        //double x_init_guess = init_guess_prepush.x;
        //double y_init_guess = init_guess_prepush.y;
        double x_init_guess = init_guess.second.x();
        double y_init_guess = init_guess.second.y();

        // ************ no init guess
        //x_init_guess = goalPose_prepush.x;
        //y_init_guess = goalPose_prepush.y;

        // Call your function:
        // (x_i, y_i, th_i, x2, y2, th2, sideAngle, R, x_init_guess, y_init_guess)
        ReloPush::OptResult optRes = ReloPush::FindPreRelocationOptimization(
            startPose.x,
            startPose.y,
            startPose.yaw,
            goalPose.x,
            goalPose.y,
            goalPose.yaw,
            sideAngle,
            R,
            x_init_guess,
            y_init_guess,
            ctx
            );

        double relocationX   = optRes.x;
        double relocationY   = optRes.y;
        double relocationYaw = optRes.landing_yaw;
        double costOpt       = optRes.cost; // cost from the solver
        double delta_yaw    = optRes.change_in_yaw;


        // for debug only
        //std::cout << "start: " << startPose.x << ", " << startPose.y << ", " << startPose.yaw << " Goal: " << goalPose.x << ", " << goalPose.y << ", " << goalPose.yaw << " th_ip: " << sideAngle << " cost: " << optRes.cost << std::endl;

        if (std::isnan(relocationYaw))
        {
            // failed to find yaw. todo: handle far points
            continue;
        }

        if(costOpt > 50)
            continue;

        // If the cost is infinite or >= bestCost, skip
        if (costOpt >= bestCost)
            continue;

        // 2b) (Optional) boundary or collision checks:
        // if (!ctx.env.inBoundary(relocationX, relocationY)) continue;
        // if (someCollisionCheck(relocationX, relocationY)) continue;

        // 2c) If your cost function doesn't already include final path cost,
        //     plan a path from (relocationX,relocationY,relocationYaw) to goalPose.
        // double dubinsDist = ...
        // double totalCost = costOpt + dubinsDist;
        double totalCost = costOpt;

        // 2d) Keep the best
        if (totalCost < bestCost)
        {
            auto temp_opt = ReloPush::OptResult(relocationX, relocationY, relocationYaw, costOpt, delta_yaw);

            //auto start_pivot = ReloPush::State(startPose.x,startPose.y,sideAngle);
            //auto startPrepush = find_pre_push(start_pivot,ctx.parameters.PrePush_dist);
            auto robot_prerelo = ReloPush::State(relocationX,relocationY,relocationYaw);
            bestDubins_prerelo = findDubins(startPose_prepush, robot_prerelo, ctx.parameters.turning_rad_pair.push);

            if(isDubinsValid(bestDubins_prerelo,ctx)!=StateValidity::valid)
                continue;

            // object PreRelo
            auto obj_prerelo = ReloPush::revert_pre_push(robot_prerelo,ctx.parameters.PrePush_dist);
            bestPreRelo_object = obj_prerelo;

            double final_push_orientation = startPose.yaw + temp_opt.change_in_yaw;
            bestPreRelo_orientation = movingObject.getNominalPose().yaw + temp_opt.change_in_yaw;
            auto final_push_pose = ReloPush::State(obj_prerelo.x,obj_prerelo.y,final_push_orientation);

            bestDubins_final = findDubins(find_pre_push(final_push_pose,ctx.parameters.PrePush_dist), find_pre_push(goalPose,ctx.parameters.PrePush_dist), ctx.parameters.turning_rad_pair.push);
            if(isDubinsValid(bestDubins_final,ctx)!=StateValidity::valid)
                continue;


            foundAny = true;
            bestCost = totalCost;
            bestOpt  = ReloPush::OptResult(relocationX, relocationY, relocationYaw, costOpt, delta_yaw);
            bestOrientationIndex = i;
            bestExtraCost = bestDubins_prerelo.lengthCost();
        }
    } // end for each orientation axis

    StateValidity out_validity = StateValidity::out_of_boundary;

    // 3) Check if we found any feasible relocation
    if (!foundAny)
    {
        // No feasible relocation found
        out_validity = StateValidity::out_of_boundary; // or another reason
    }

    else
    {
        auto planApproach = check_approach_validity(bestDubins_prerelo.targetState, bestDubins_final.startState, movingObject, bestOrientationIndex, final_push_index, bestOpt.change_in_yaw, ctx);

        if(planApproach->validity == PlanValidity::success)
        {
            // 4) If found, create an edge in the graph
            Edge e;
            bool inserted;
            boost::tie(e, inserted) = boost::add_edge(v1, v2, g);
            if (inserted)
            {
                g[e].weight = bestCost;
                g[e].mode   = ConnectionMode::PRE_RELOCATION;
                g[e].preRelo.used            = true;
                g[e].preRelo.xRelocated_robot      = bestOpt.x;
                g[e].preRelo.yRelocated_robot      = bestOpt.y;
                g[e].preRelo.yawReloacted_robot = bestPreRelo_orientation;
                g[e].preRelo.xRelocated_object = bestPreRelo_object.x;
                g[e].preRelo.yRelocated_object = bestPreRelo_object.y;
                g[e].preRelo.yawRelocated_object = bestPreRelo_object.yaw;
                g[e].preRelo.extraCost       = bestExtraCost; // all pushing is included in the weight
                g[e].preRelo.relocatingIndex = bestOrientationIndex;
                g[e].preRelo.relocatingIndex= bestOrientationIndex;
                g[e].preRelo.reason = reason_in;

                EdgePath preReloPath(true, bestDubins_prerelo);
                EdgePath appPath(false, planApproach->getPathPtr(true));
                EdgePath finalPushPath(true, bestDubins_final);
                g[e].paths = {std::make_shared<EdgePath>(preReloPath), std::make_shared<EdgePath>(appPath), std::make_shared<EdgePath>(finalPushPath)};;
                g[e].srcVertexData = g[v1];
                g[e].sinkVertexData = g[v2];

                // Possibly store the reason in 'reason_in' or g[e].preRelo.reason
                // g[e].preRelo.reason = reason_in;
                out_validity = StateValidity::valid;
            }
        }
    }

    ctx.addObs(startPose);
    if(data2.type==VertexType::OBJECT_VERTEX)
        ctx.addObs(goalPose);

    // for debug
    //auto obs_after = ctx.env.get_obs();

    //if(obs_before.size() != obs_after.size())
    //    std::cout << "!!!!" << std::endl;

    return out_validity;
}




bool addEdge(Graph &g, Vertex v1, Vertex v2, PlanningContext &ctx)
{
    const auto &data1 = g[v1];
    const auto &data2 = g[v2];

    //for debug only
    bool deb = false;
    if(data1.name=="box1" && data2.name=="goal2")
        deb = true;

    if(data1.name != data2.name)
    {
        // try normal mode
        StateValidity normalEdge = addEdgeNormalMode(g, v1, v2, ctx);

        // Normal mode Failed
        if(normalEdge != StateValidity::valid)
        {
            // find pre-relocation
            StateValidity preRelocationEdge = StateValidity::out_of_boundary;

            if(!ctx.use_prelo_optimization)
                preRelocationEdge = addEdgePrerelocation(g,v1,v2,ctx, normalEdge);
            else
                preRelocationEdge = addEdgePrerelocation_Optimization(g,v1,v2,ctx, normalEdge);

            if(preRelocationEdge == StateValidity::valid)
                return true;
        }

        else
        {
            return true;
        }
    }
    // skip for same object


    // If both checks fail, we do NOT add any edge
    return false;
}

// -----------------------------------------------------------------
// Print graph info (vertices + edges)
// -----------------------------------------------------------------
void printGraphInfo(const Graph &g)
{
    using vertex_iter = boost::graph_traits<Graph>::vertex_iterator;
    vertex_iter vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi)
    {
        Vertex v = *vi;
        const auto &vd = g[v];
        std::cout << "Vertex " << v << " (" << vd.name << ")\n";

        using out_edge_iter = boost::graph_traits<Graph>::out_edge_iterator;
        out_edge_iter ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(v, g); ei != ei_end; ++ei)
        {
            Edge e = *ei;
            const auto &ed = g[e];

            // Convert connection mode to a string
            std::string modeStr;
            switch(ed.mode)
            {
            case ConnectionMode::NORMAL_MODE:    modeStr = "normalMode"; break;
            case ConnectionMode::PRE_RELOCATION: modeStr = "preRelocation"; break;
            default:                             modeStr = "none";
            }

            Vertex tgt = boost::target(e, g);
            std::cout << "  -> Edge to vertex " << tgt
                      << ", weight=" << ed.weight
                      << ", mode=" << modeStr << "\n";
        }
        std::cout << std::endl;
    }
}


// -----------------------------------------------------------------
// Create multiple vertices for one object
// -----------------------------------------------------------------
std::vector<Vertex> createVerticesForObject(Graph &g, const ObjectInfo &obj)
{
    std::vector<Vertex> createdVertices;

    // If object has zero or negative sides, treat it as 1 (fallback)
    int sides = (obj.numberOfSides > 0) ? obj.numberOfSides : 1;

    for (int i = 0; i < sides; ++i)
    {
        Vertex v = addVertexToGraph(
            g,
            VertexType::OBJECT_VERTEX,
            obj.name,
            i,                         // orientationIndex
            obj.nominalOrientation,
            obj.x,
            obj.y,
            obj.numberOfSides
            );
        createdVertices.push_back(v);
    }

    return createdVertices;
}

// -----------------------------------------------------------------
// Create multiple vertices for one goal
// -----------------------------------------------------------------
std::vector<Vertex> createVerticesForGoal(Graph &g, const GoalInfo &goal)
{
    std::vector<Vertex> createdVertices;

    // If goal has zero or negative sides, treat it as 1 (fallback)
    int sides = (goal.numberOfSides > 0) ? goal.numberOfSides : 1;

    for (int i = 0; i < sides; ++i)
    {
        Vertex v = addVertexToGraph(
            g,
            VertexType::GOAL_VERTEX,
            goal.name,
            i,                          // orientationIndex
            goal.nominalOrientation,
            goal.x,
            goal.y,
            goal.numberOfSides
            );
        createdVertices.push_back(v);
    }

    return createdVertices;
}

// void connectVertexSets(Graph &g,
//                        const std::vector<Vertex> &fromSet,
//                        const std::vector<Vertex> &toSet, const PlanningParameters &params)
// {
//     for (auto v1 : fromSet)
//     {
//         for (auto v2 : toSet)
//         {
//             addEdge(g, v1, v2, params); // v1 -> v2
//         }
//     }
// }

void initGraph(Graph& g, ObjectMap& objects, GoalMap& goals)
{
    // We might store boundary in a global or pass it around as needed
    // For each object, create vertices (each orientation)
    std::vector<std::vector<Vertex>> objectVerts;
    objectVerts.reserve(objects.size());
    for (auto &oi : objects)
    {
        // Create vertices
        auto verts = createVerticesForObject(g, oi.second);
        // Also, if your VertexData has a 'radius' field, set it there
        // (In createVerticesForObject, you likely do it automatically)
        objectVerts.push_back(verts);
    }

    // Same for goals
    std::vector<std::vector<Vertex>> goalVerts;
    goalVerts.reserve(goals.size());
    for (auto &gi : goals)
    {
        auto verts = createVerticesForGoal(g, gi.second);
        // If createVerticesForGoal sets g[v].radius = gi.enclosingRadius,
        // then you have it in the graph
        goalVerts.push_back(verts);
    }
}

/*
void initGraph(Graph& g, ObjectList& objects, GoalList& goals)
{
    // We might store boundary in a global or pass it around as needed
    // For each object, create vertices (each orientation)
    std::vector<std::vector<Vertex>> objectVerts;
    objectVerts.reserve(objects.size());
    for (auto &oi : objects)
    {
        // Create vertices
        auto verts = createVerticesForObject(g, oi);
        // Also, if your VertexData has a 'radius' field, set it there
        // (In createVerticesForObject, you likely do it automatically)
        objectVerts.push_back(verts);
    }

    // Same for goals
    std::vector<std::vector<Vertex>> goalVerts;
    goalVerts.reserve(goals.size());
    for (auto &gi : goals)
    {
        auto verts = createVerticesForGoal(g, gi);
        // If createVerticesForGoal sets g[v].radius = gi.enclosingRadius,
        // then you have it in the graph
        goalVerts.push_back(verts);
    }
}
*/

/**
 * @brief Build edges among multiple objects and goals, ensuring
 *        goals have no outgoing edges.
 *
 * @param g             The graph
 * @param objectVerts   objectVerts[i] = all orientation-vertices for the i-th object
 * @param goalVerts     goalVerts[j]   = all orientation-vertices for the j-th goal
 */
/*
void buildAllEdges(
    Graph &g,
    const std::vector<std::vector<Vertex>> &objectVerts,
    const std::vector<std::vector<Vertex>> &goalVerts, const PlanningParameters &params)
{
    // 1) Connect objects among themselves (object -> object)
    //    For each pair of distinct sets i, j, connect both ways or one way, depending on your logic.
    //    If you want to allow object->object edges in both directions, you could call connectVertexSets
    //    twice (once from i->j, once from j->i).
    //    But let's assume you only want a single direction or you want to unify them. It's up to you.
    for (size_t i = 0; i < objectVerts.size(); i++)
    {
        for (size_t j = i + 1; j < objectVerts.size(); j++)
        {
            // object i -> object j
            connectVertexSets(g, objectVerts[i], objectVerts[j], params);
            // object j -> object i, if you want edges in both directions
            connectVertexSets(g, objectVerts[j], objectVerts[i], params);
        }
    }

    // 2) Connect objects to goals (object -> goal)
    //    But do NOT connect the other way around, because goals have no outgoing edges.
    for (auto &oSet : objectVerts)
    {
        for (auto &gSet : goalVerts)
        {
            connectVertexSets(g, oSet, gSet, params);
        }
    }

    // 3) No goal->goal edges (since goals have no outgoing edges).
    //    So we do not connect goal sets among themselves.
}
*/

/**
 * @brief Returns all vertices in the graph whose type == OBJECT_VERTEX.
 */
std::vector<Vertex> getAllObjectVertices(const Graph &g)
{
    std::vector<Vertex> objectVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::OBJECT_VERTEX)
        {
            objectVerts.push_back(v);
        }
    }
    return objectVerts;
}

/**
 * @brief Returns all vertices in the graph whose type == GOAL_VERTEX.
 */
std::vector<Vertex> getAllGoalVertices(const Graph &g)
{
    std::vector<Vertex> goalVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::GOAL_VERTEX)
        {
            goalVerts.push_back(v);
        }
    }
    return goalVerts;
}

/**
 * @brief Returns all vertices in the graph that match
 *        - type == OBJECT_VERTEX
 *        - name == objectName
 */
std::vector<Vertex> getObjectVertices(const Graph &g, const std::string &objectName)
{
    std::vector<Vertex> objectVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::OBJECT_VERTEX && vData.name == objectName)
        {
            objectVerts.push_back(v);
        }
    }
    return objectVerts;
}

/**
 * @brief Returns all vertices in the graph that match
 *        - type == GOAL_VERTEX
 *        - name == goalName
 */
std::vector<Vertex> getGoalVertices(const Graph &g, const std::string &goalName)
{
    std::vector<Vertex> goalVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::GOAL_VERTEX && vData.name == goalName)
        {
            goalVerts.push_back(v);
        }
    }
    return goalVerts;
}

/**
 * @brief Build edges by scanning the entire graph for object vertices and goal vertices,
 *        then connecting them.
 *
 *        - All object vertices are stored in 'objectList'
 *        - All goal vertices in 'goalList'
 *        - Then object->object and object->goal edges are created,
 *          but goals have no outgoing edges.
 *
 * @param g    The graph (already containing object and goal vertices).
 * @param ctx  Your PlannerContext (or environment + parameters).
 */
void buildAllEdges(Graph &g, PlanningContext ctx)
{
    // 0) Init env with obstacles
    std::unordered_set<ReloPush::State> obs;
    for(auto& it : ctx.mo_list)
    {
        obs.insert(ReloPush::State(it.second.x, it.second.y, it.second.nominalOrientation));
        //obs.insert(State(it.get_x(),it.get_y(),0));
    }

    for(auto& it : ctx.delivered_list)
    {
        obs.insert(ReloPush::State(it.second.x, it.second.y, it.second.nominalOrientation));
        //obs.insert(State(it.get_x(),it.get_y(),0));
    }
    ctx.updateObs(obs);


    // 1) Separate object vs. goal vertices
    std::vector<Vertex> objectVerts;
    std::vector<Vertex> goalVerts;

    // Get iterators for all vertices in g
    auto vertsPair = boost::vertices(g);  // returns (begin, end)
    for (auto it = vertsPair.first; it != vertsPair.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::OBJECT_VERTEX)
        {
            objectVerts.push_back(v);
        }
        else if (vData.type == VertexType::GOAL_VERTEX)
        {
            goalVerts.push_back(v);
        }
    }

    // 2) Connect objects among themselves (object->object).
    //    You can do object->object in both directions or just one direction.
    for (size_t i = 0; i < objectVerts.size(); ++i)
    {
        for (size_t j = i + 1; j < objectVerts.size(); ++j)
        {

            // object i -> object j
            addEdge(g, objectVerts[i], objectVerts[j], ctx);
            // object j -> object i
            addEdge(g, objectVerts[j], objectVerts[i], ctx);
        }
    }

    // 3) Connect objects to goals (object->goal), but NOT goal->object
    for (auto vObj : objectVerts)
    {
        for (auto vGoal : goalVerts)
        {
            addEdge(g, vObj, vGoal, ctx);
        }
    }

    // 4) No goal->goal edges (since goals have no outgoing edges).
    //    So we do nothing for goal->goal.
}


// -----------------------------------------------------------------
// Visualize Graph
// -----------------------------------------------------------------
void writeGraphToDot(const Graph &g, const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << " for writing.\n";
        return;
    }

    // We create lambdas that describe how to write properties for vertices/edges.
    auto vertexWriter = [&](std::ostream &out, const Vertex v)
    {
        const auto &vd = g[v];
        // Create a label string. For example:
        //   name (orientationIndex)
        //   position: (x,y)
        //   or anything else you’d like to display!
        out << "[label=\""
            << vd.name << "(" << vd.orientationIndex << ")\\n"
            << "pos=(" << vd.x << "," << vd.y << ")\\n"
            << "ori=" << vd.getActualOrientation()
            << "\"]";
    };

    auto edgeWriter = [&](std::ostream &out, const Edge e)
    {
        // Show the edge weight in the label
        out << "[label=\"" << g[e].weight << "\"]";
    };

    // Write to .dot
    boost::write_graphviz(file, g, vertexWriter, edgeWriter);

    file.close();
    std::cout << "Wrote graph to " << filename << "\n";
}

// A helper function to write a .dot file that includes node positions.
void writeGraphWithCoordinates(const Graph &g, const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << "\n";
        return;
    }

    double scale = 100.0;  // Increase to spread nodes more

    auto vertexWriter = [&](std::ostream &out, Vertex v)
    {
        const auto &vd = g[v];
        out << "["
            << "label=\"" << vd.name << "\\n"
            << "ori=" << vd.getActualOrientation() << "\" "
            << "fontsize=\"10\" "
            << "shape=\"circle\" width=\"0.7\" fixedsize=\"true\" "
            << "style=\"filled\" fillcolor=\""
            << (vd.type == VertexType::OBJECT_VERTEX ? "lightblue" : "yellow") << "\" "
            << "pos=\"" << (scale * vd.x) << ","
            << (scale * vd.y) << "!\""
            << "]";
    };

    auto edgeWriter = [&](std::ostream &out, Edge e)
    {
        out << "[label=\"" << g[e].weight << "\"]";
    };

    auto graphWriter = [&](std::ostream &out)
    {
        out << "graph [layout=neato, overlap=false, splines=line];\n";
    };

    boost::write_graphviz(file, g, vertexWriter, edgeWriter, graphWriter);
    file.close();
    std::cout << "Wrote scaled graph to " << filename << "\n";
}

