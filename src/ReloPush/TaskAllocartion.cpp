#include <ReloPush/TaskAllocation.hpp>
#include <ReloPush/ReloPushBossDiagnostics.hpp>
#include <tuple>

EdgeMatrixEntry PairCostResult::getBestPath()
{
    // return matrixResult->pathMat->at(bestRow)[bestCol];
    return matrixResult->getBestPathMatEntry();
}

/*
void PairCostResult::remove_top(void)
{
    if(matrixResult->sortedEntries.size()==0)
        return;

    auto tmp = matrixResult->sortedEntries[0];
    // remove from cost matrix
    matrixResult->costMat(tmp.row,tmp.col) = std::numeric_limits<double>::infinity();
    // if any left
    if (!matrixResult->sortedEntries.empty()) {
        // remove from sortedEntries
        matrixResult->sortedEntries.erase(matrixResult->sortedEntries.begin());
        // update next best
        //bestCost = matrixResult->sortedEntries[0].cost;
        //bestRow = matrixResult->sortedEntries[0].row;
        //bestCol = matrixResult->sortedEntries[0].col;
    }
}
*/

void PairCostResult::remove_top(void)
{
    if (matrixResult->sortedEntries.empty())
        return;

    auto tmp = matrixResult->sortedEntries.front();
    matrixResult->sortedEntries.erase(matrixResult->sortedEntries.begin());
    matrixResult->costMat(tmp.row, tmp.col) = std::numeric_limits<double>::infinity();

    // Optional: Verify consistency
    assert(matrixResult->sortedEntries.empty() ||
           matrixResult->sortedEntries.front().cost == matrixResult->costMat(matrixResult->sortedEntries.front().row, matrixResult->sortedEntries.front().col));
}

void PathsToSinglePath(std::vector<EdgeData> &paths, std::vector<size_t> &path_sizes,
                       ReloPush::StatePath &out_path, double interpolation_resolution)
{
    for (auto &it : paths)
    {
        for (auto it2 : it.paths)
        {
            ReloPush::StatePathPtr statePath;
            // Check if the variant holds a StatePathPtr
            if (std::holds_alternative<ReloPush::StatePathPtr>(it2->path))
            {
                statePath = std::get<ReloPush::StatePathPtr>(it2->path);
            }
            // If needed, handle reloDubinsPath here (currently ignored)
            else if (std::holds_alternative<reloDubinsPath>(it2->path))
            {
                auto dubinsPath = std::get<reloDubinsPath>(it2->path);
                statePath = dubinsPath.interpolate(interpolation_resolution); // todo: parse map resolution
            }

            // fill out_path
            for (auto &p : *statePath)
            {
                out_path.push_back(p);
            }
            // count size
            path_sizes.push_back(statePath->size());
        }
    }
}

ReloPush::StatePathPtr EdgePathListToSinglePath(EdgePathList paths, double resolution)
{
    ReloPush::StatePath out_path;
    for (auto it : *paths)
    {
        ReloPush::StatePathPtr statePath;
        // Check if the variant holds a StatePathPtr
        if (std::holds_alternative<ReloPush::StatePathPtr>(it.path))
        {
            statePath = std::get<ReloPush::StatePathPtr>(it.path);
        }
        // If needed, handle reloDubinsPath here (currently ignored)
        else if (std::holds_alternative<reloDubinsPath>(it.path))
        {
            auto dubinsPath = std::get<reloDubinsPath>(it.path);
            statePath = dubinsPath.interpolate(resolution); // todo: parse map resolution
        }

        // fill out_path
        for (auto &p : *statePath)
        {
            out_path.push_back(p);
        }
    }

    return std::make_shared<ReloPush::StatePath>(out_path);
}

double FinalAllocation::getPushingLength(void) const
{
    double obsPush = 0;
    double taskPush = 0;

    // sum obs push
    for (auto &it : *obsReloPaths)
    {
        if (it.is_pushing)
            obsPush += it.getLength();
    }

    for (auto &it : paths) // for each edge
    {
        for (auto &it2 : it.paths) // for each path
        {
            if (it2->is_pushing)
                taskPush += it2->getLength();
        }
    }

    return obsPush + taskPush;
}

std::pair<ReloPush::StatePathPtr, std::vector<size_t>> FinalAllocation::toSinglePathPtr(double interpolation_resolution)
{
    ReloPush::StatePath obs_path(0);
    ReloPush::StatePath out_path(0);

    // add ObsRelo
    // for(size_t n=0; n<obsReloPaths->size(); n++)
    //{
    auto pathPtr = EdgePathListToSinglePath(obsReloPaths, 0.2);
    obs_path.insert(obs_path.end(), pathPtr->begin(), pathPtr->end());
    //}

    std::vector<size_t> path_sizes = {obs_path.size()};
    PathsToSinglePath(paths, path_sizes, out_path, interpolation_resolution);

    ReloPush::StatePath combined;
    // Reserve space for performance (optional).
    combined.reserve(firstApproachPath->size() + obs_path.size() + out_path.size());

    combined.insert(combined.end(), firstApproachPath->begin(), firstApproachPath->end());
    combined.insert(combined.end(), obs_path.begin(), obs_path.end());
    combined.insert(combined.end(), out_path.begin(), out_path.end());

    return std::make_pair(std::make_shared<ReloPush::StatePath>(combined), path_sizes);
}

std::tuple<ReloPush::StatePathPtr, std::vector<size_t>, std::vector<bool>>
FinalAllocation::toSinglePathPtrWithTypes(double interpolation_resolution)
{
    ReloPush::StatePath combined;
    std::vector<size_t> path_sizes;
    std::vector<bool> path_is_transfer;

    auto append_segment = [&](const ReloPush::StatePathPtr &segment, bool is_transfer)
    {
        if (!segment || segment->empty())
        {
            return;
        }

        combined.insert(combined.end(), segment->begin(), segment->end());
        path_sizes.push_back(segment->size());
        path_is_transfer.push_back(is_transfer);
    };

    append_segment(firstApproachPath, false);

    if (obsReloPaths)
    {
        for (const auto &obs_path : *obsReloPaths)
        {
            auto state_path = obs_path.toStatePath(0.2);
            append_segment(state_path, obs_path.is_pushing);
        }
    }

    for (auto &edge : paths)
    {
        for (const auto &edge_path : edge.paths)
        {
            if (!edge_path)
            {
                continue;
            }
            auto state_path = edge_path->toStatePath(interpolation_resolution);
            append_segment(state_path, edge_path->is_pushing);
        }
    }

    return std::make_tuple(std::make_shared<ReloPush::StatePath>(combined), path_sizes, path_is_transfer);
}

int FinalAllocation::countObsRelo()
{
    return obsReloUpdate.size();
}

int FinalAllocation::countPreRelo()
{
    int count = 0;
    for (auto &it : paths)
    {
        if (it.mode == ConnectionMode::PRE_RELOCATION)
        {
            count++;
        }
    }

    return count;
}

// Normalize angle to [-pi, pi)
static double normalizeAngle(double theta)
{
    while (theta > M_PI)
        theta -= 2.0 * M_PI;
    while (theta <= -M_PI)
        theta += 2.0 * M_PI;
    return theta;
}

// Compute difference between angles, normalized
static double angleDiff(double a, double b)
{
    return normalizeAngle(a - b);
}

// Check if the segment from prev to curr is forward
bool isForwardSegment(const ReloPush::State &prev, const ReloPush::State &curr)
{
    double dx = curr.x - prev.x;
    double dy = curr.y - prev.y;
    double seg_angle = std::atan2(dy, dx);
    double heading_diff = std::fabs(angleDiff(seg_angle, prev.yaw));
    const double THRESHOLD = M_PI / 2.0; // 90 degrees
    return (heading_diff <= THRESHOLD);
}

/**
 * @brief Generate a timed trajectory from an ordered list of States.
 *
 * If state[i].is_push==true, we assume the robot always drives forward,
 * using velocity = v_p. If is_push==false, we determine forward/backward
 * by comparing the waypoint’s yaw to the displacement vector. In that case,
 * use velocity = v_np for forward, velocity = -v_backward for backward.
 *
 * @param path            Input sequence of waypoints without timing.
 * @param v_p             Forward pushing velocity (e.g. 0.4).
 * @param v_np            Forward non-pushing velocity (e.g. 0.5).
 * @param v_backward      Backward velocity magnitude (e.g. 0.25).
 * @return A new path with time values assigned in each ReloPush::State.
 */
ReloPush::StatePath generateTimedTrajectory(const ReloPush::StatePath &path,
                                            float v_p,
                                            float v_np,
                                            float v_backward, bool is_pushing)
{
    ReloPush::StatePath timedPath;
    if (path.empty())
    {
        return timedPath;
    }

    // Prepare output container with same size
    timedPath.resize(path.size());
    // Copy the first state and initialize its time
    timedPath[0] = path[0];
    timedPath[0].time = 0.0f;

    float time_offset = 0.0f;
    bool is_prev_forward = true;

    // Iterate over each pair of consecutive waypoints
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        bool add_wait = false;

        const auto &s1 = timedPath[i];
        auto &s2 = timedPath[i + 1];

        // Copy next state from input
        s2 = path[i + 1];

        // Calculate displacement from s1 to s2
        float dx = s2.x - s1.x;
        float dy = s2.y - s1.y;
        float dist = std::sqrt(dx * dx + dy * dy);

        // Determine whether it’s forward or backward
        // by projecting (dx, dy) onto s1’s heading vector.
        float headingX = std::cos(s1.yaw);
        float headingY = std::sin(s1.yaw);
        float dot = dx * headingX + dy * headingY; // if dot>0 => forward; else backward

        // Decide velocity
        float chosenVel = 0.0f;
        if (is_pushing)
        {
            // For pushing, always drive forward at v_p
            chosenVel = v_p;
        }
        else
        {
            // Non-pushing: forward = v_np, backward = -v_backward
            if (dot >= 0.0f)
            {
                chosenVel = v_np; // forward
                if (!is_prev_forward)
                    add_wait = true;
                is_prev_forward = true;
            }
            else
            {
                chosenVel = v_backward; // backward (negative velocity)
                if (is_prev_forward)
                    add_wait = true;
                is_prev_forward = false;
            }
        }

        // Compute travel time for segment i->i+1
        // (Use absolute value for distance ÷ speed)
        float speed = std::fabs(chosenVel);
        float dt = (speed > 1e-6f) ? (dist / speed) : 0.0f;

        if (add_wait)
            time_offset = 0.8;
        else
        {
            time_offset = 0;
        }

        // Accumulate the time in s2
        s2.time = s1.time + dt + time_offset;
        // Assign speed
        timedPath[i].vel = chosenVel;
        // Assign is_push
        timedPath[i].is_pushing = is_pushing;
    }

    return timedPath;
}

ReloPush::trajectory_elem state2trajelem(ReloPush::State &s, float time_off = 0)
{
    ReloPush::trajectory_elem out_elem(s.x, s.y, s.yaw, s.vel, s.time + time_off, s.is_pushing);
    return out_elem;
}

ReloPush::trajectory statePath2traj(ReloPush::StatePathPtr sp,
                                    float v_p, float v_np, float v_backward,
                                    bool is_pushing)
{

    auto p = generateTimedTrajectory(*sp, v_p, v_np, v_backward, is_pushing); // todo: add is_pushing during graph gen

    ReloPush::trajectory out_traj;
    for (auto &it : p)
    {
        out_traj.append_waypoint(state2trajelem(it));
    }
    // push more
    if (is_pushing)
    {
        /*
        auto last_wpt = p.back();
        auto push_more = ReloPush::revert_pre_push(last_wpt,0.05); //todo: parese from param

        auto last_t = out_traj.trajectory_points->back().time;
        auto last_v = out_traj.trajectory_points->back().ref_vel;

        // swap vel
        out_traj.trajectory_points->back().ref_vel = v_p;

        auto new_t = sqrt(pow(push_more.x-last_wpt.x,2) + pow(push_more.y-last_wpt.y,2))/v_p + last_t;

        ReloPush::trajectory_elem p_more(push_more.x,push_more.y,push_more.yaw,last_v,new_t,is_pushing);
        out_traj.append_waypoint(p_more);
        */
    }

    return out_traj;
}

ReloPush::trajectory FinalAllocation::genTrajectory(double interpolation_resolution)
{
    // todo: parse these from param
    float v_p = 0.28;
    float v_np = 0.35;
    float v_backward = -0.3;

    /*

    // add approach
    auto app_traj = statePath2traj(firstApproachPath,v_p,v_np,v_backward,false);

    std::vector<ReloPush::trajectory> trajs;

    for(auto& it : paths) // for each edge (possibly multiple if obs-relocated)
    {
        for(auto& ep : it.paths) // for each edgepath (one or more for each edge)
        {
            auto temp_traj = statePath2traj(ep->toStatePath(interpolation_resolution),v_p,v_np,v_backward,ep->is_pushing);
            trajs.push_back(temp_traj);
        }
    }


    */

    std::vector<ReloPush::trajectory> trajs;

    // first approach
    auto app_traj = statePath2traj(firstApproachPath, v_p, v_np, v_backward, false);

    // obstacle relocation
    for (auto &op : *obsReloPaths)
    {
        auto obsPath = op.toStatePath();
        // offset for real robot correction
        auto landing = obsPath->back();
        // for debug only
        // std::cout << landing << std::endl;
        auto landing_off = landing.get_postPush(Constants::obs_relo_offset);
        obsPath->at(obsPath->size() - 1) = landing_off;
        // std::cout << obsPath->back() << std::endl;
        trajs.push_back(statePath2traj(obsPath, v_p, v_np, v_backward, op.is_pushing));
    }

    // main pushing
    // for(auto& p : paths) // each edge-path
    for (size_t i = 0; i < paths.size(); i++)
    {
        auto &p = paths[i];
        for (size_t n = 0; n < p.paths.size(); n++) // multiple if prerelocation
        {
            auto edgePath = p.paths[n]->toStatePath();
            // additional push for prerelocation
            if (p.paths.size() > 1 && n == 0)
            {
                auto addPush = edgePath->back().get_postPush(Constants::additional_push_dist);
                edgePath->push_back(addPush);
            }
            trajs.push_back(statePath2traj(edgePath, v_p, v_np, v_backward, p.paths[n]->is_pushing));
        }
        // transit between edges (exists sometimes)
        if (edgeTransitPaths.size() > i && edgeTransitPaths.size() > 0)
        {
            auto edgeTrans = edgeTransitPaths[i];
            trajs.push_back(statePath2traj(edgeTrans, v_p, v_np, v_backward, false));
        }
    }

    // augment trajectories one by one
    for (auto &it : trajs)
    {
        app_traj.augment_trajectory(it);
    }

    // return
    return app_traj;
}

std::vector<ReloPush::State> FinalTaskSequence::to_StateList(void)
{
    std::vector<ReloPush::State> out_list(task_sequence.size());

    for (size_t n = 0; n < task_sequence.size(); n++)
        out_list[n] = ReloPush::State(task_sequence[n].goal.x, task_sequence[n].goal.y, task_sequence[n].goal.nominalOrientation);

    return out_list;
}

ReloPush::trajectory FA2Trajectory(std::vector<FinalAllocation> &fa)
{
    ReloPush::trajectory out_traj;
    for (auto &it : fa)
    {
        auto temp = it.genTrajectory();
        out_traj.augment_trajectory(temp);
    }
    return out_traj;
}

/**
 * @brief Scans an Eigen::MatrixXd for its minimal value (if any).
 *        Returns (value, row, col). If matrix is empty, returns +inf.
 */
MatrixMinEntry findMatrixMin(const Eigen::MatrixXd &mat)
{
    MatrixMinEntry result;
    result.value = std::numeric_limits<double>::infinity();
    result.row = -1;
    result.col = -1;

    int rows = mat.rows();
    int cols = mat.cols();
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            double val = mat(r, c);
            if (val < result.value)
            {
                result.value = val;
                result.row = r;
                result.col = c;
            }
        }
    }
    return result;
}

/**
 * @brief Builds a cost matrix of size (objectVerts.size() x goalVerts.size()),
 *        where entry (i,j) = shortest-path distance from objectVerts[i] to goalVerts[j].
 *
 *        Also builds a sorted list of (row, col, cost) in ascending order of cost.
 *
 * @param g           The graph
 * @param objectVerts The vertices corresponding to an object
 * @param goalVerts   The vertices corresponding to a goal
 * @return A MatrixResult struct containing:
 *         - costMat: the NxM Eigen matrix
 *         - sortedEntries: a list of (row, col, cost) sorted ascending by cost
 */
/*
MatrixResult computeCostMatrix(
    const Graph &g,
    const std::vector<Vertex> &objectVerts,
    const std::vector<Vertex> &goalVerts)
{
    // Let Nobj = number of object vertices, Ngoal = number of goal vertices
    size_t Nobj  = objectVerts.size();
    size_t Ngoal = goalVerts.size();

    // Create an NxM matrix
    Eigen::MatrixXd costMatrix(Nobj, Ngoal);

    // Initialize to +infinity (optional if you prefer to fill directly)
    costMatrix.setConstant(std::numeric_limits<double>::infinity());

    // For each object vertex, run Dijkstra to find shortest paths to all other vertices
    for (size_t i = 0; i < Nobj; ++i)
    {
        Vertex src = objectVerts[i];

        // Distances to every vertex in the graph from 'src'
        std::vector<double> distMap(boost::num_vertices(g), std::numeric_limits<double>::infinity());

        // The index map
        auto indexMap = get(boost::vertex_index, g);

        // Run Dijkstra
        boost::dijkstra_shortest_paths(
            g,
            src,
            boost::distance_map(boost::make_iterator_property_map(distMap.begin(), indexMap))
                .weight_map(get(&EdgeData::weight, g)));

        // For each goal vertex j, store the distance in costMatrix(i, j)
        for (size_t j = 0; j < Ngoal; ++j)
        {
            Vertex goalV = goalVerts[j];
            double d = distMap[goalV];
            costMatrix(i, j) = d;
        }
    }

    // Now we have the cost matrix filled.
    // Let's create the sorted list of (row, col, cost).
    std::vector<RowColCost> rowColList(0);
    //rowColList.reserve(Nobj * Ngoal);

    for (int i = 0; i < costMatrix.rows(); ++i)
    {
        for (int j = 0; j < costMatrix.cols(); ++j)
        {
            double c = costMatrix(i, j);
            // You could ignore +inf entries or keep them—your choice.
            RowColCost rcc { i, j, c };
            // push only when the cost is not inf
            if(c < 1000000000) // todo: use inf
                rowColList.push_back(rcc);
        }
    }

    // Sort ascending by cost
    std::sort(
        rowColList.begin(),
        rowColList.end(),
        [](const RowColCost &a, const RowColCost &b)
        {
            return std::tie(a.cost, a.row, a.col) <
                   std::tie(b.cost, b.row, b.col);
        }
        );

    // Build final result
    MatrixResult result;
    result.costMat       = costMatrix;
    result.sortedEntries = rowColList;

    return result;
}
*/

void sortByDistance(std::vector<ReloPush::State> &goals, const ReloPush::State &start)
{
    std::sort(goals.begin(), goals.end(), [&start](const ReloPush::State &a, const ReloPush::State &b)
              { return StateDistance(start, a) < StateDistance(start, b); });
}

ReloPush::StatePathPtr Find_ObsRelo(ObjectInfo &mo, PlanningContext &ctx, std::vector<EdgeData> &edgesInfo)
{
    auto init_pusing_poses = mo.getPushingPoses();

    std::vector<ReloPush::State> found_candidates(0);

    ReloPush::State objectPos(init_pusing_poses[0].x, init_pusing_poses[0].y, init_pusing_poses[0].yaw);
    for (const auto &pp : init_pusing_poses)
    {
        // Direction is represented as a pair of (dx, dy)
        double dx = cosf(pp.yaw) * ctx.parameters.map_resolution;
        double dy = sinf(pp.yaw) * ctx.parameters.map_resolution; // unit vector

        bool out_of_boundary = false;
        // Check positions along this direction
        ReloPush::State obsrelo_candidate = pp;
        while (!out_of_boundary)
        {
            obsrelo_candidate.x += dx;
            obsrelo_candidate.y += dy;

            // auto validity = env.stateValid(obsrelo_candidate,Constants::carWidth,2*Constants::obsRadius);
            StateValidity validity = StateValidity::valid;

            ObjectMap obsMap = ctx.env_push.get_obs();
            auto obs = obsMap.toStateList();
            // add path points as obstacles to avoid overlap with path
            std::vector<ReloPush::State> pathObs;
            std::vector<size_t> path_sizes; // dummy
            PathsToSinglePath(edgesInfo, path_sizes, pathObs, ctx.parameters.obs_rad * 2);
            obs.insert(obs.end(), pathObs.begin(), pathObs.end());

            for (auto &it : obs)
            {
                if (StateDistance(it, obsrelo_candidate) < Constants::obsRadius * 2 + Constants::LF_nonpush + Constants::LB + 0.05)
                {
                    validity = StateValidity::collision;
                    break;
                }
                else if (obsrelo_candidate.x < ctx.parameters.boundary.xMin || obsrelo_candidate.x > ctx.parameters.boundary.xMax || obsrelo_candidate.y < ctx.parameters.boundary.yMin || obsrelo_candidate.y > ctx.parameters.boundary.yMax)
                {
                    validity = StateValidity::out_of_boundary;
                    break;
                }
            }

            // out-of-bounday: finish with this vec
            if (validity == StateValidity::out_of_boundary)
            {
                out_of_boundary = true;
                break;
            }
            // obsrelo candidate found
            else if (validity == StateValidity::valid)
                break;
        }

        // not out-of-bounday: found a candidate
        if (!out_of_boundary)
            found_candidates.push_back(obsrelo_candidate);
    }

    // sort
    sortByDistance(found_candidates, objectPos);

    // If no valid relocation found, return the original position
    return std::make_shared<ReloPush::StatePath>(found_candidates);
}

/*
MatrixResult computeCostMatrixWithPaths(
    const Graph &g,
    const std::vector<Vertex> &objectVerts,
    const std::vector<Vertex> &goalVerts)
{
    // 1) Setup matrix dimension
    size_t Nobj  = objectVerts.size();
    size_t Ngoal = goalVerts.size();

    // 2) Initialize cost matrix
    Eigen::MatrixXd costMatrix(Nobj, Ngoal);
    costMatrix.setConstant(std::numeric_limits<double>::infinity());

    // 3) Dijkstra: fill costMatrix exactly as before
    for (size_t i = 0; i < Nobj; ++i)
    {
        Vertex src = objectVerts[i];

        // Distances to all vertices
        std::vector<double> distMap(boost::num_vertices(g), std::numeric_limits<double>::infinity());
        auto indexMap = get(boost::vertex_index, g);

        boost::dijkstra_shortest_paths(
            g,
            src,
            boost::distance_map(boost::make_iterator_property_map(distMap.begin(), indexMap))
                .weight_map(get(&EdgeData::weight, g)));

        for (size_t j = 0; j < Ngoal; ++j)
        {
            Vertex goalV = goalVerts[j];
            double d = distMap[goalV];
            costMatrix(i, j) = d;
        }
    }

    // 4) Prepare the MatrixResult
    MatrixResult result;
    result.costMat = costMatrix;

    // 5) Initialize pathMat (Nobj x Ngoal), each cell is an empty EdgePathList
    result.pathMat.resize(Nobj);
    for (size_t i = 0; i < Nobj; ++i)
    {
        result.pathMat[i].resize(Ngoal);
        // Each pathMat[i][j] is *by default* an empty EdgePathList
    }

    // 6) We'll build sortedEntries from the cost matrix
    std::vector<RowColCost> rowColList;

    for (int i = 0; i < costMatrix.rows(); ++i)
    {
        for (int j = 0; j < costMatrix.cols(); ++j)
        {
            double c = costMatrix(i, j);
            // We'll consider c < 1e9 as a "finite" cost
            if (c < 1e9)
            {
                // 6a) Insert into rowColList
                RowColCost rcc { i, j, c };
                rowColList.push_back(rcc);

                // 6b) Find the edge in the graph
                Vertex vObj  = objectVerts[i];
                Vertex vGoal = goalVerts[j];

                Edge e;
                bool hasEdge;
                boost::tie(e, hasEdge) = boost::edge(vObj, vGoal, g);
                if (hasEdge)
                {
                    // If g[e].paths is a vector<EdgePath>,
                    // we store a *copy* of each path in pathMat[i][j].
                    const auto &edgePaths = g[e].paths;
                    result.pathMat[i][j].clear();
                    for (auto &ep : edgePaths)
                    {
                        // Make a shared_ptr
                        auto epPtr = std::make_shared<EdgePath>(ep);
                        result.pathMat[i][j].push_back(epPtr);
                    }
                }
            }
        }
    }

    // 7) Sort ascending by cost
    std::sort(rowColList.begin(), rowColList.end(),
              [](const RowColCost &a, const RowColCost &b)
              {
                  return std::tie(a.cost, a.row, a.col) <
                         std::tie(b.cost, b.row, b.col);
              });

    // Fill in the final sorted list
    result.sortedEntries = rowColList;

    return result;
}
*/
// previous version

struct VertexNameFilter
{
    using vertex_descriptor = Graph::vertex_descriptor;

    // Default ctor: only used to create the "end" iterator
    VertexNameFilter()
        : g(nullptr), src(), name() {}

    // Real ctor: store graph ptr and the src-vertex’s name
    VertexNameFilter(const Graph *g_, vertex_descriptor s)
        : g(g_), src(s), name((*g_)[s].name) {}

    bool operator()(vertex_descriptor v) const
    {
        // In the "end" iterator case, g==nullptr → treat everything as allowed
        if (!g)
            return true;
        // Always allow the source; otherwise exclude same-name vertices
        return v == src || (*g)[v].name != name;
    }

private:
    const Graph *g;
    vertex_descriptor src;
    std::string name;
};

MatrixResultPtr computeCostMatrixWithPaths(
    const Graph &g,
    const std::vector<Vertex> &objectVerts,
    const std::vector<Vertex> &goalVerts,
    PlanningContext &ctx)
{
    // 1) Dimensions
    size_t Nobj = objectVerts.size();
    size_t Ngoal = goalVerts.size();

    // 2) Initialize cost matrix
    Eigen::MatrixXd costMatrix(Nobj, Ngoal);
    costMatrix.setConstant(std::numeric_limits<double>::infinity());

    // We'll store final data in this 'MatrixResult'
    auto result = std::make_shared<MatrixResult>();
    result->costMat = costMatrix;

    // 3) Prepare the NxM pathMat
    auto pathMatPtr = std::make_shared<EdgeDataPathMatrix>();
    pathMatPtr->resize(Nobj);
    for (size_t i = 0; i < Nobj; ++i)
    {
        (*pathMatPtr)[i].resize(Ngoal);
        // each cell is an EdgeMatrixEntry with edgesInfo = {}
    }

    // We'll fill sortedEntries at the end
    SortedEntryList rowColList;

    auto indexMap = get(boost::vertex_index, g);
    size_t numV = boost::num_vertices(g);

    // 4) For each object vertex i, run Dijkstra
    for (size_t i = 0; i < Nobj; ++i)
    {
        Vertex src = objectVerts[i];

        // build a default-constructible filter that knows about 'g' and 'src'
        VertexNameFilter vf(&g, src);
        auto fg = boost::make_filtered_graph(
            g,
            boost::keep_all(), // no edge filtering
            vf);               // our named functor

        // Distances & predecessors
        std::vector<double> distMap(numV, std::numeric_limits<double>::infinity());
        std::vector<Vertex> predMap(numV, Graph::null_vertex());

        boost::dijkstra_shortest_paths(
            fg, src, // g, src,
            boost::distance_map(boost::make_iterator_property_map(distMap.begin(), indexMap))
                .predecessor_map(boost::make_iterator_property_map(predMap.begin(), indexMap))
                .weight_map(get(&EdgeData::weight, g))); // todo: skip unnecessary search

        // 4b) For each goal vertex j, reconstruct the path if finite
        for (size_t j = 0; j < Ngoal; ++j)
        {
            Vertex goalV = goalVerts[j];
            double d = distMap[indexMap[goalV]];

            result->costMat(i, j) = d;
            if (d < 1e9) // finite
            {

                // Reconstruct path from (src -> goalV)
                // We'll gather a list of EdgeDataPathPair
                EdgeMatrixEntry edgesInfo;

                Vertex cur = goalV;
                while (cur != src && cur != Graph::null_vertex())
                {
                    Vertex p = predMap[indexMap[cur]];
                    if (p == Graph::null_vertex() || p == cur)
                    {
                        edgesInfo.edgesInfo.clear();
                        break;
                    }

                    // The edge is p->cur
                    Edge e;
                    bool hasEdge;
                    boost::tie(e, hasEdge) = boost::edge(p, cur, g);
                    if (hasEdge)
                    {
                        EdgeData ed = g[e];
                        // copy the entire EdgeData
                        // ed = g[e];
                        // Now copy all EdgePaths from g[e].paths
                        // std::vector<EdgePath> temp_list;
                        // for (auto &ep : g[e].paths)
                        //{
                        // auto epPtr = std::make_shared<EdgePath>(ep);
                        //    temp_list.push_back(*ep);
                        //}
                        // pair.edgeData.paths = std::make_shared<std::vector<EdgePath>>(temp_list);
                        edgesInfo.edgesInfo.push_back(std::move(ed));
                    }
                    cur = p;
                }

                // The 'edgesInfo' is reversed (goal->...->src).
                // If you want them in forward order (src->...->goal), reverse:
                std::reverse(edgesInfo.edgesInfo.begin(), edgesInfo.edgesInfo.end());

                (*pathMatPtr)[i][j].obsReloList.clear();
                // handle multiple edges
                if (edgesInfo.edgesInfo.size() > 1)
                {
                    for (size_t n = 1; n < edgesInfo.edgesInfo.size(); n++)
                    {
                        // pivot object
                        auto pivotObj = ctx.mo_list[edgesInfo.edgesInfo[n].srcVertexData.name];
                        // for each object
                        auto obsRelo_candidates = Find_ObsRelo(pivotObj, ctx, edgesInfo.edgesInfo);

                        // handle failure in finding ObsRelo
                        if (obsRelo_candidates->size() == 0)
                        {
                            edgesInfo.edgesInfo.clear();
                            result->costMat(i, j) = std::numeric_limits<double>::infinity();
                            break;
                        }

                        // use first candidate
                        auto obsRelo_state = obsRelo_candidates->at(0);
                        ReloPush::State start_state = ReloPush::State(pivotObj.x, pivotObj.y, obsRelo_state.yaw);

                        double obs_push_d = ReloPush::StateDistance(start_state, obsRelo_state);
                        result->costMat(i, j) += obs_push_d;

                        // ReloPush::State start_pre_push = find_pre_push(start_state, params::pre_push_dist);
                        // ReloPush::State goal_pre_push = find_pre_push(obsRelo_state, params::pre_push_dist+params::pre_relo_pre_push_offset);

                        //(*pathMatPtr)[i][j].obsReloList.push_back(std::make_pair(start_pre_push, goal_pre_push));
                        edgesInfo.obsReloList.push_back(std::make_pair(start_state, obsRelo_state));
                    }
                }

                // Build vertexChain
                std::vector<VertexData> chain;
                if (!edgesInfo.edgesInfo.empty())
                {
                    chain.reserve(edgesInfo.edgesInfo.size() + 1); // optional performance
                    chain.push_back(edgesInfo.edgesInfo[0].srcVertexData);
                    for (auto &ed : edgesInfo.edgesInfo)
                    {
                        chain.push_back(ed.sinkVertexData);
                    }
                }

                // Build RowColCost
                RowColCost rcc;
                rcc.row = static_cast<int>(i);
                rcc.col = static_cast<int>(j);
                rcc.cost = result->costMat(i, j);
                rowColList.push_back(rcc);

                // 4c) Store in pathMat: i.e. the entire path of edges from i->j
                (*pathMatPtr)[i][j] = std::move(edgesInfo);
                (*pathMatPtr)[i][j].vertexChain = std::move(chain);
            }
        } // end for j
    } // end for i

    // 5) Sort rowColList by cost
    std::sort(rowColList.begin(), rowColList.end(),
              [](const RowColCost &a, const RowColCost &b)
              {
                  return std::tie(a.cost, a.row, a.col) <
                         std::tie(b.cost, b.row, b.col);
              });

    // 6) Fill in the final sorted list & pathMat
    result->sortedEntries = std::move(rowColList);
    result->pathMat = pathMatPtr;

    return result;
}

/*
void pick_best(std::vector<ObjectGoalPair>& allObjectGoalPairs, Graph& g)
{
    double bestCost = std::numeric_limits<double>::infinity();
    std::string bestObjName, bestGoalName;
    int bestObjVertexIdx = -1;
    int bestGoalVertexIdx = -1;

    for (auto &pair : allObjectGoalPairs)
    {
        // Suppose pair.objectName and pair.goalName identify them
        std::vector<Vertex> objVerts = getObjectVertices(g, pair.objectName);
        std::vector<Vertex> goalVerts = getGoalVertices(g, pair.goalName);

        Eigen::MatrixXd costMat = computeCostMatrix(g, objVerts, goalVerts);

        // Now find the min entry
        for (int i = 0; i < costMat.rows(); ++i)
        {
            for (int j = 0; j < costMat.cols(); ++j)
            {
                double costVal = costMat(i, j);
                if (costVal < bestCost)
                {
                    bestCost = costVal;
                    bestObjName = pair.objectName;
                    bestGoalName = pair.goalName;
                    bestObjVertexIdx  = i;
                    bestGoalVertexIdx = j;
                }
            }
        }
    }

    // bestCost holds the lowest cost among *all* pairs
    std::cout << "Lowest cost: " << bestCost << " from object=" << bestObjName
              << " (vertex idx=" << bestObjVertexIdx << ")"
              << " to goal=" << bestGoalName
              << " (vertex idx=" << bestGoalVertexIdx << ")" << std::endl;
}
*/

/*
std::vector<PairCostResult> computeAndSortAllPairs(
    const Graph &g,
    std::unordered_map<std::string, ObjectGoalPair> &pairs)
{
    std::vector<PairCostResult> results;
    results.reserve(pairs.size());

    for (auto &p : pairs)
    {
        // 1) Get object vertices, goal vertices
        std::vector<Vertex> objVerts  = getObjectVertices(g, p.second.objectName);
        std::vector<Vertex> goalVerts = getGoalVertices(g, p.second.goalName);

        // 2) Build the cost matrix (which also produces sorted entries)
        //MatrixResult matrixRes = computeCostMatrix(g, objVerts, goalVerts);
        MatrixResult matrixRes = computeCostMatrixWithPaths(g, objVerts, goalVerts);

        // 3) The minimal cost entry is sortedEntries[0], unless the matrix is empty
        double bestCost  = std::numeric_limits<double>::infinity();
        int bestRow      = -1;
        int bestCol      = -1;

        if (!matrixRes.sortedEntries.empty())
        {
            // sortedEntries[0] is the smallest cost
            const auto &top = matrixRes.sortedEntries[0];
            bestCost = top.cost;
            bestRow  = top.row;
            bestCol  = top.col;
        }

        // 4) Create a PairCostResult
        PairCostResult pcr;
        pcr.objectName  = p.second.objectName;
        pcr.goalName    = p.second.goalName;
        pcr.bestCost    = bestCost;
        pcr.bestRow     = bestRow;
        pcr.bestCol     = bestCol;
        pcr.matrixResult= std::make_shared<MatrixResult>(matrixRes);

        results.push_back(pcr);
    }

    // 5) Sort results by bestCost ascending
    std::sort(
        results.begin(),
        results.end(),
        [](const PairCostResult &a, const PairCostResult &b)
        {
            return a.bestCost < b.bestCost;
        }
        );

    return results;
}
*/

std::map<std::string, PairCostResult> computeMatrixPairs(
    const Graph &g, std::unordered_map<std::string, ObjectGoalPair> &pairs, PlanningContext &ctx)
{
    std::map<std::string, PairCostResult> resultMap;

    // We iterate over the 'pairs' map, which is keyed by objectName.
    // Each value is an ObjectGoalPair that has (objectName, goalName).
    for (auto &p : pairs)
    {
        // e.g. p.first is the objectName as a key in the unordered_map
        //      p.second is the ObjectGoalPair with objectName, goalName
        const auto &objName = p.second.objectName;
        const auto &goalName = p.second.goalName;

        // 1) Get object vertices, goal vertices
        std::vector<Vertex> objVerts = getObjectVertices(g, objName);
        std::vector<Vertex> goalVerts = getGoalVertices(g, goalName);

        // 2) Build the cost matrix (which also produces sorted entries + pathMat)
        MatrixResultPtr matrixRes = computeCostMatrixWithPaths(g, objVerts, goalVerts, ctx);

        // 3) The minimal cost entry is sortedEntries[0], unless the matrix is empty
        double bestCost = std::numeric_limits<double>::infinity();
        int bestRow = -1;
        int bestCol = -1;

        if (!matrixRes->sortedEntries.empty())
        {
            const auto &top = matrixRes->sortedEntries[0];
            bestCost = top.cost;
            bestRow = top.row;
            bestCol = top.col;
        }

        // 4) Create a PairCostResult
        PairCostResult pcr;
        pcr.objectName = objName;
        pcr.goalName = goalName;
        pcr.input_order = p.second.input_order;
        // pcr.bestCost     = bestCost;
        // pcr.bestRow      = bestRow;
        // pcr.bestCol      = bestCol;
        //  store the entire MatrixResult in a shared_ptr
        pcr.matrixResult = matrixRes;

        // 5) Insert into the map with objectName as key
        resultMap[objName] = pcr;
    }

    return resultMap;
}

/**
 * @brief Finds the single lowest cost among all PairCostResult entries,
 *        returning its details (object, goal, row, col, cost, and index).
 *
 * @param results The vector of PairCostResult from computeAndSortAllPairs().
 * @return A LowestCostInfo with the absolute minimal cost found.
 *         If 'results' is empty, fields will be default/invalid.
 */

LowestCostInfo findAbsoluteLowestCost(std::map<std::string, PairCostResult> &resultsMap)
{
    LowestCostInfo best;
    best.cost = std::numeric_limits<double>::infinity();
    best.row = -1;
    best.col = -1;

    // For each pair in the map (key=object name, value=PairCostResult)
    for (auto &kv : resultsMap)
    {
        const auto &pcr = kv.second;     // pcr is a PairCostResult
        auto mResPtr = pcr.matrixResult; // the MatrixResultPtr

        if (!mResPtr) // no matrix result? skip
            continue;

        // 1) Check all RowColCost in sortedEntries
        //    Each entry is (row, col, cost), sorted ascending, but we must
        //    look at them all because a "second best" in one pair might still
        //    be lower than the "best" in another pair.
        for (auto &rcc : mResPtr->sortedEntries) // todo: it is already sorted. only need to compare the first of each
        {
            if (rcc.cost < best.cost)
            {
                best.cost = rcc.cost;
                best.row = rcc.row;
                best.col = rcc.col;
                best.objectName = pcr.objectName;
                best.goalName = pcr.goalName;
            }
        }
    }

    return best;
}

/* Exhaustive search on matrices
LowestCostInfo findAbsoluteLowestCost(std::map<std::string, PairCostResult> &resultsMap)
{
    LowestCostInfo best;
    PairCostResult res;
    best.cost          = std::numeric_limits<double>::infinity();
    //best.indexInArray  = -1;  // or remove if not needed
    best.row           = -1;
    best.col           = -1;

    // Iterate over the map: key is std::string (object name), value is PairCostResult
    for (const auto &kv : resultsMap)
    {
        // kv.first  is the object name
        // kv.second is the PairCostResult
        const auto &p = kv.second;
        double c = p.bestCost;
        if (c < best.cost)
        {
            best.cost       = c;
            // Instead of best.indexInArray, we just store -1 or omit
            best.objectName = p.objectName;
            best.goalName   = p.goalName;
            best.row        = p.bestRow;
            best.col        = p.bestCol;

            res = kv.second;
        }
    }

    return best;
}
*/

// ---------------------------------------------------------------------------
// Helper Function 2: Attempt a single relocation plan segment
//
// This function handles the repeated logic:
//   1) Remove old obstacle
//   2) Add new obstacle
//   3) Attempt path planning
//   4) If fail, mark cost ∞ and revert environment changes
//   5) If success, record the path and update 'obsReloPathList'
// ---------------------------------------------------------------------------
PathPlanResultPtr attemptObsRelocation(PlanningContext &planCtx,
                                       const ReloPush::State &fromState_prepush,
                                       const ReloPush::State &toState_prepush,
                                       ReloPush::State &fromObs, // obstacle to remove
                                       ReloPush::State &toObs,   // obstacle to add
                                       PairResultsMap &pairResults,
                                       const LowestCostInfo &bestPick,
                                       std::vector<EdgePath> &ObsReloPathList,
                                       std::unordered_map<std::string, ReloPush::State> &ToUpdate,
                                       const ObjectInfo &pivotObjInfo, // ObsRelo Object
                                       const ReloPush::State &objNewState)
{

    auto saved_obs = planCtx.env_nonpush.get_obs(); // double checking

    planCtx.checkObsCount("\to1");
    // 1) Environment updates
    // planCtx.removeObs(fromObs);
    planCtx.removeObs(pivotObjInfo.name);
    // planCtx.addObs(toObs);
    planCtx.addObs(ObjectInfo(pivotObjInfo, toObs));

    auto before_obs = planCtx.env_push.get_obs();
    // 2) Attempt path planning (after obs relo)
    planCtx.checkObsCount("\to2");
    auto res = planHybridAstar(fromState_prepush, toState_prepush, planCtx, true);
    planCtx.checkObsCount("\to3");
    if (res->validity != PlanValidity::success)
    {
        planCtx.checkObsCount("\to4");
        // revert environment changes
        // planCtx.addObs(fromObs);
        planCtx.removeObs(pivotObjInfo.name);
        planCtx.addObs(ObjectInfo(pivotObjInfo, fromObs));
        // planCtx.removeObs(toObs);

        planCtx.checkObsCount("\to5");

        auto after_obs = planCtx.env_push.get_obs();
        planCtx.checkObsCount("\to6");
        if (before_obs != after_obs)
        {
            std::cerr << "Environment not properly reverted!" << std::endl;
            // Add detailed prints here to identify discrepancies
        }
        planCtx.checkObsCount("\to7");

        planCtx.updateObs(saved_obs);

        return res;
    }

    // 3) If success, record the path
    ReloPush::StatePath pre_pair_path = {
        find_pre_push(fromObs, planCtx.parameters.PrePush_dist),
        fromState_prepush};
    // The first EdgePath is a trivial “push” from some pre-push position
    ObsReloPathList.push_back(EdgePath(/*isPrePush=*/true, std::make_shared<ReloPush::StatePath>(pre_pair_path)));
    // The second EdgePath is the actual planned path
    ObsReloPathList.push_back(EdgePath(/*isPrePush=*/false, res->getPathPtr(true)));

    // 4) Update the object’s new location
    ToUpdate[pivotObjInfo.name] = objNewState;

    return res;
}

// ---------------------------------------------------------------------------
// Helper Function 3: One iteration of picking the best pair and planning
// ---------------------------------------------------------------------------

bool findFeasibleAllocation_old(PairResultsMap &pairResults,
                                const std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                                PlanningContext &planCtx,
                                std::vector<EdgePath> &ObsReloPathList,
                                LowestCostInfo &bestPick,
                                std::unordered_map<std::string, ReloPush::State> &ToUpdate,
                                std::string &failedObjectName, ObjectMap objects, EdgeMatrixEntry &bestMatEntry, ReloPush::StatePathPtrList &transitPaths)
{
    // Attempt to find the absolute lowest cost
    bestPick = findAbsoluteLowestCost(pairResults);

    if (bestPick.row == -1 || bestPick.col == -1)
    {
        std::cout << "No feasible pair found (or no pairs left)!\n";
        return false;
    }

    if (bestPick.cost == std::numeric_limits<double>::infinity())
    {
        std::cout << "No feasible pair found (cost=∞)!\n";
        failedObjectName = bestPick.objectName; // Let caller handle
        return false;
    }

    // retrieve matrixResult for this specific object
    auto &bestPairEntry = pairResults[bestPick.objectName];

    // The path chain, including any intermediate obstacle relocations
    bestMatEntry = bestPairEntry.matrixResult->getBestPathMatEntry();

    // 1) Plan the intermediate obs-relocations, if any
    for (size_t obs = 1; obs < bestMatEntry.obsReloList.size(); obs++)
    {
        auto pivotObj = bestMatEntry.vertexChain[obs];
        auto prev_pair = bestMatEntry.obsReloList[obs - 1];
        auto next_pair = bestMatEntry.obsReloList[obs];

        auto fromState = prev_pair.second;
        auto toState = next_pair.first;
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        auto toState_pre = find_pre_push(toState, planCtx.parameters.PrePush_dist);

        // check start valid
        auto sv = planCtx.env_push.stateValid(fromState_pre);
        if (!sv)
            return false;

        // Attempt relocation
        auto res = attemptObsRelocation(planCtx,
                                        fromState_pre, toState_pre,
                                        prev_pair.first,  // obs to remove
                                        prev_pair.second, // obs to add
                                        pairResults, bestPick,
                                        ObsReloPathList,
                                        ToUpdate,
                                        pivotObj.toObjectInfo(),
                                        fromState);
        if (res->validity != PlanValidity::success)
        {
            // If we fail, the cost is set to ∞ for that pair, so we return false
            failedObjectName = bestPick.objectName;
            return false;
        }
    }

    // 2) Plan from the last relocated obstacle to the final push
    if (!bestMatEntry.obsReloList.empty())
    {
        // last relocation pair
        auto last_pair = bestMatEntry.obsReloList.back();
        auto fromState = last_pair.second;
        auto best_obj = objects[bestPick.objectName]; // object to deliver
        // final approach
        auto final_approach_obs = ReloPush::State(best_obj.x,
                                                  best_obj.y,
                                                  best_obj.getOrientation(bestPick.row));
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        // auto toState_pre   = find_pre_push(final_approach_obs, planCtx.parameters.PrePush_dist);
        auto toState_pre = bestPairEntry.matrixResult->getBestPathMatEntry().edgesInfo[0].paths[0]->getFirstWaypoint(); // picked by sorted entries
        // check start valid
        auto sv = planCtx.env_push.stateValid(fromState_pre);
        if (sv.get_validity() == StateValidity::out_of_boundary)
            return false;

        auto res = attemptObsRelocation(planCtx,
                                        fromState_pre, toState_pre,
                                        last_pair.first,
                                        last_pair.second,
                                        pairResults, bestPick,
                                        ObsReloPathList,
                                        ToUpdate,
                                        bestMatEntry.vertexChain[bestMatEntry.vertexChain.size() - 2].toObjectInfo(),
                                        fromState);
        if (res->validity != PlanValidity::success)
        {
            failedObjectName = bestPick.objectName;
            return false;
        }
    }

    // For multiple edges, check and plan transit between the two
    if (bestMatEntry.edgesInfo.size() > 1)
    {
        transitPaths.clear();
        for (size_t n = 1; n < bestMatEntry.edgesInfo.size(); n++)
        {
            auto last_goal = bestMatEntry.edgesInfo[n - 1].paths.back()->getLastWaypoint();
            auto this_start = bestMatEntry.edgesInfo[n].paths.front()->getFirstWaypoint();

            auto res = planHybridAstar(last_goal, this_start, planCtx, true);

            if (res->validity != PlanValidity::success)
            {
                return false;
            }
            else
            {
                // add transit path
                transitPaths.push_back(res->getPathPtr(true));
            }
        }
    }

    return true;
}

bool findFeasibleAllocation(PairResultsMap &pairResults,
                            const std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                            PlanningContext &planCtx,
                            std::vector<EdgePath> &ObsReloPathList,
                            LowestCostInfo &candidate,
                            std::unordered_map<std::string, ReloPush::State> &ToUpdate,
                            std::string &failedObjectName, ObjectMap objects, EdgeMatrixEntry &bestMatEntry, ReloPush::StatePathPtrList &transitPaths,
                            ReloPush::State &robot, ReloPush::StatePathPtr &firstApp)
{

    PlanningContext planCtx_backup = planCtx;
    auto obs_backup = planCtx.env_nonpush.get_obs();

    // Attempt to find the absolute lowest cost
    candidate = findAbsoluteLowestCost(pairResults);

    if (candidate.row == -1 || candidate.col == -1)
    {
        std::cout << "No feasible pair found (or no pairs left)!\n";
        return false;
    }

    if (candidate.cost == std::numeric_limits<double>::infinity())
    {
        std::cout << "No feasible pair found (cost=∞)!\n";
        failedObjectName = candidate.objectName; // Let caller handle
        return false;
    }

    // retrieve matrixResult for this specific object
    auto &bestPairEntry = pairResults[candidate.objectName];

    // The path chain, including any intermediate obstacle relocations
    bestMatEntry = bestPairEntry.matrixResult->getBestPathMatEntry();

    // std::vector<EdgePath> ObsReloPathList;
    // std::unordered_map<std::string, ReloPush::State> ToUpdate;
    // ReloPush::StatePathPtrList transitPaths;

    ReloPush::State firstAppGoal;

    std::unordered_map<std::string, ReloPush::State> obsReloUpdate;

    // Plan obs relocations (if two or more). Transit paths between obsRelo
    for (size_t obs = 1; obs < bestMatEntry.obsReloList.size(); obs++)
    {
        auto pivotObj = bestMatEntry.vertexChain[obs];
        auto pivotObjInfo = pivotObj.toObjectInfo();
        auto this_pair = bestMatEntry.obsReloList[obs - 1];
        auto next_pair = bestMatEntry.obsReloList[obs];

        auto fromState = this_pair.second; // end of prev obs relo (pivot)
        auto toState = next_pair.first;    // start of next obs relo
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        auto toState_pre = find_pre_push(toState, planCtx.parameters.PrePush_dist);

        if (!planCtx.env_push.stateValid(fromState_pre))
            return false;

        auto res = attemptObsRelocation(planCtx, fromState_pre, toState_pre, this_pair.first, this_pair.second,
                                        pairResults, candidate, ObsReloPathList, ToUpdate, pivotObjInfo, fromState);
        if (res->validity != PlanValidity::success)
            return false;

        // Obs Relo ok
        obsReloUpdate.insert(std::make_pair(pivotObjInfo.name, this_pair.second));
    }

    // Plan from last relocation to final push (if any)
    // 1. Robot to first obsRelo start [robot -> firstObsStart]
    // 2. Last obsRelo start to pushing start [lastObsStart -> FirstWaypoint]
    if (!bestMatEntry.obsReloList.empty())
    {

        auto last_pair = bestMatEntry.obsReloList.back();
        auto first_pair = bestMatEntry.obsReloList.front();

        auto fromState = last_pair.second;
        auto &best_obj = objects[candidate.objectName];
        auto final_approach_obs = ReloPush::State(best_obj.x, best_obj.y, best_obj.getOrientation(candidate.row));
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        auto toState_pre = bestPairEntry.matrixResult->getBestPathMatEntry().edgesInfo[0].paths[0]->getFirstWaypoint();

        if (planCtx.env_push.stateValid(fromState_pre).get_validity() == StateValidity::out_of_boundary)
            return false;

        planCtx.checkObsCount("\t2");

        auto res_lastobs = attemptObsRelocation(planCtx, fromState_pre, toState_pre, last_pair.first, last_pair.second,
                                                pairResults, candidate, ObsReloPathList, ToUpdate,
                                                bestMatEntry.vertexChain[bestMatEntry.vertexChain.size() - 2].toObjectInfo(),
                                                fromState); // last obs relo to first wpt
        if (res_lastobs->validity != PlanValidity::success)
            return false;

        // Obs Relo ok
        obsReloUpdate.insert(std::make_pair(bestMatEntry.vertexChain[bestMatEntry.vertexChain.size() - 2].name, last_pair.second));

        // update first approach goal
        firstAppGoal = find_pre_push(first_pair.first, planCtx.parameters.PrePush_dist); // todo: start of first pair
    }

    // no Obstacle Relocation
    else
    {
        // auto obj_info = objects[candidate.objectName];
        // ReloPush::State obj_start = obj_info.getPushingPose(candidate.row);
        // ReloPush::State obj_start_pre = find_pre_push(obj_start,planCtx.parameters.PrePush_dist);

        firstAppGoal = bestMatEntry.edgesInfo[0].paths.at(0)->getFirstWaypoint();
    }

    // Plan approach (transit)
    // planCtx.checkObsCount("\t3");
    // auto res_app = planHybridAstar(robot, obj_start_pre, planCtx, true);

    auto res_app = planHybridAstar(robot, firstAppGoal, planCtx_backup, true); // before any obs relocation
    if (res_app->validity != PlanValidity::success)
    {
        /*
        // try once more with more margins
        auto onceMoreRobot = find_pre_push(robot,0.1);
        auto onceMoreGoal = find_pre_push(firstAppGoal,0.1);

        auto res_app_om = planHybridAstar(onceMoreRobot, onceMoreGoal,planCtx,true);
        {
            if(res_app_om->validity != PlanValidity::success)
            {
                // restore obstacles
                planCtx.updateObs(obs_backup);
                return false;
            }
            else
            {
                res_app = res_app_om;
            }
        }
*/
        planCtx.updateObs(obs_backup);
        return false;
    }
    // transitPaths.push_back(res_app->getPathPtr(true));
    firstApp = res_app->getPathPtr(true); // store it to allocation if evertying is fine

    // planCtx.checkObsCount("\t4");
    //  Plan transit paths between edges if needed. (if next edge has pre-relocation, it will likely to have different start)
    if (bestMatEntry.edgesInfo.size() > 1)
    {
        transitPaths.clear();
        for (size_t n = 1; n < bestMatEntry.edgesInfo.size(); n++)
        {
            auto last_goal = bestMatEntry.edgesInfo[n - 1].paths.back()->getLastWaypoint();
            auto this_start = bestMatEntry.edgesInfo[n].paths.front()->getFirstWaypoint();

            // inter-vertex movement (object moved to next vertex)
            auto prev_vertex = bestMatEntry.edgesInfo[n - 1].srcVertexData;
            auto next_vertex = bestMatEntry.edgesInfo[n - 1].sinkVertexData;

            auto prev_obj = prev_vertex.toObjectInfo();
            auto next_obj = next_vertex.toObjectInfo();

            // temporary change in obstacles
            auto moved_obj = next_obj;
            moved_obj.name = prev_obj.name;
            planCtx.removeObs(prev_obj);
            planCtx.addObs(moved_obj);

            auto res = planHybridAstar(last_goal, this_start, planCtx, true);
            if (res->validity != PlanValidity::success)
            {
                // restore obstacles
                planCtx.updateObs(obs_backup);
                return false;
            }

            // put back
            planCtx.removeObs(moved_obj);
            planCtx.addObs(prev_obj);

            transitPaths.push_back(res->getPathPtr(true));
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Helper Function 4a: The main planning/allocation loop
// ---------------------------------------------------------------------------

bool performAllocations_old(const WorkspaceBoundary &boundary,
                            ObjectMap &objects,
                            ObjectMap &goals,
                            std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                            std::vector<FinalAllocation> &finalSequence,
                            bool &use_opt)
{
    GoalMap delivered_objs;

    while (!objGoalPairs.empty())
    {
        std::cout << "\n============================\n"
                  << "Remaining pairs: " << objGoalPairs.size() << "\n";

        // (a) Build the graph from scratch
        Graph g;
        initGraph(g, objects, goals);

        // (b) Setup PlanningParameters and context
        PlanningParameters params;
        params.boundary = boundary;
        PlanningContext planCtx(params, objects, delivered_objs, use_opt, false); // todo: add no_init_guess

        // (d) Build edges
        buildAllEdges(g, planCtx);
        const int diag_depth = static_cast<int>(finalSequence.size());
        ReloPushBossDiagnostics::log_search_state(
            diag_depth, objects, goals, objGoalPairs, delivered_objs, ReloPush::State(), g);

        // (e) Compute cost matrices for all remaining pairs
        auto pairResults = computeMatrixPairs(g, objGoalPairs, planCtx);
        ReloPushBossDiagnostics::log_pair_results(diag_depth, pairResults);

        // We'll store info about the best pick
        LowestCostInfo bestPick;
        std::vector<EdgePath> ObsReloPathList;
        std::unordered_map<std::string, ReloPush::State> ToUpdate;
        EdgeMatrixEntry bestMatEntry;
        ReloPush::StatePathPtrList transitPaths;

        // Take a snapshot of the planning context (for FinalAllocation)
        PlanningContext ctxSnapshot(planCtx);

        // Start searching for a feasible solution
        bool isFeasible = false;
        while (!isFeasible)
        {
            isFeasible = findFeasibleAllocation_old(pairResults, objGoalPairs,
                                                    planCtx, ObsReloPathList,
                                                    bestPick, ToUpdate, bestPick.objectName, objects, bestMatEntry, transitPaths);
            if (isFeasible)
                break;

            // If no feasible solution, break or handle failure
            if (bestPick.row == -1 || bestPick.cost == std::numeric_limits<double>::infinity())
            {
                std::cerr << "Failure: no feasible solution found for any pair.\n";
                return false;
            }
            else
            {
                // update matrix and find next best
                // approach failed. Adjust cost matrix for re-planning.
                pairResults[bestPick.objectName].matrixResult->sortedEntries.erase(pairResults[bestPick.objectName].matrixResult->sortedEntries.begin()); // pop the first
                pairResults[bestPick.objectName].matrixResult->costMat(bestPick.row, bestPick.col) = std::numeric_limits<double>::infinity();             // mark inf on cost matrix
                continue;                                                                                                                                 // try other options
            }
        }

        // If we found a valid bestPick, commit it:
        //  Update object positions that got relocated
        for (const auto &pair : ToUpdate)
        {
            std::cout << "Relocated: " << pair.first
                      << " => " << pair.second << std::endl;

            objects[pair.first].x = pair.second.x;
            objects[pair.first].y = pair.second.y;
            // orientation if needed ...
        }

        // Mark the chosen object as delivered
        delivered_objs[bestPick.objectName] = goals[bestPick.goalName];

        // Print the best pair
        std::cout << "BEST PAIR => " << bestPick.objectName
                  << " -> " << bestPick.goalName
                  << ", cost=" << bestPick.cost
                  << ", row=" << bestPick.row
                  << ", col=" << bestPick.col << "\n";

        // Build the FinalAllocation entry
        FinalAllocation chosen;
        chosen.object = objects[bestPick.objectName];
        chosen.goal = goals[bestPick.goalName];
        chosen.cost = bestPick.cost;
        chosen.row = bestPick.row;
        chosen.col = bestPick.col;
        chosen.vertexChain = bestMatEntry.vertexChain;
        chosen.edgeTransitPaths = transitPaths;

        chosen.startPose = ReloPush::State(chosen.object.x, chosen.object.y,
                                           chosen.object.getOrientation(chosen.row));
        chosen.goalPose = ReloPush::State(chosen.goal.x, chosen.goal.y,
                                          chosen.goal.getOrientation(chosen.col));

        // chosen.paths = pairResults[bestPick.objectName].getBestPath().edgesInfo;
        chosen.paths = pairResults[bestPick.objectName].matrixResult->getBestPathMatEntry().edgesInfo;
        chosen.obsReloPaths = std::make_shared<std::vector<EdgePath>>(ObsReloPathList);
        chosen.snapshot = ctxSnapshot;

        finalSequence.push_back(chosen);

        // (h) Remove the chosen pair so we don’t pick it again
        objGoalPairs.erase(bestPick.objectName);
        objects.erase(bestPick.objectName);
        goals.erase(bestPick.goalName);
    }
    return true;
}

// Helper function to print the current allocation state.
void printCurrentState(const std::vector<FinalAllocation> &finalSequence,
                       const std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                       const GoalMap &delivered_objs)
{
    std::cout << "\n---------- Current State ----------" << std::endl;
    std::cout << "Final Sequence:" << std::endl;
    for (const auto &alloc : finalSequence)
    {
        std::cout << "  " << alloc.object.name << " -> "
                  << alloc.goal.name << " (cost=" << alloc.cost << ")" << std::endl;
    }

    std::cout << "Remaining Object-Goal Pairs:" << std::endl;
    for (const auto &pair : objGoalPairs)
    {
        std::cout << "  " << pair.first << " -> " << pair.second.goalName << std::endl;
    }

    std::cout << "Delivered Objects:" << std::endl;
    for (const auto &deliv : delivered_objs)
    {
        std::cout << "  " << deliv.first << " delivered to " << deliv.second.name << std::endl;
    }
    std::cout << "-----------------------------------\n"
              << std::endl;
}

// Helper: Extract and sort all remaining allocation candidates by cost
std::vector<LowestCostInfo> getSortedPairCandidates(const PairResultsMap &pairResults)
{
    std::vector<LowestCostInfo> candidates;
    for (const auto &kv : pairResults)
    {
        const auto &pcr = kv.second;
        for (const auto &entry : pcr.matrixResult->sortedEntries)
        {
            if (entry.cost != std::numeric_limits<double>::infinity())
            {
                LowestCostInfo info;
                info.objectName = pcr.objectName;
                info.goalName = pcr.goalName;
                info.input_order = pcr.input_order;
                info.row = entry.row;
                info.col = entry.col;
                info.cost = entry.cost;
                candidates.push_back(info);
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const LowestCostInfo &a, const LowestCostInfo &b)
              {
                  return std::tie(a.cost, a.input_order, a.objectName, a.goalName, a.row, a.col) <
                         std::tie(b.cost, b.input_order, b.objectName, b.goalName, b.row, b.col);
              });
    return candidates;
}

// Helper: Try a specific allocation, set outAllocation if successful
bool tryAllocation(
    const LowestCostInfo &candidate,
    PairResultsMap &pairResults,
    PlanningContext &planCtx,
    std::unordered_map<std::string, ObjectInfo> &objects,
    std::unordered_map<std::string, GoalInfo> &goals,
    std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
    GoalMap &delivered_objs,
    ReloPush::State &robot,
    FinalAllocation &outAllocation)
{

    // for debug
    // bool deb = (candidate.objectName == "b1");
    // std::cout << "[debug] " << candidate.objectName << std::endl;
    auto obs_backup = planCtx.env_nonpush.get_obs();
    PlanningContext planCtx_backup = planCtx;

    planCtx.checkObsCount("\t1");
    // Pull out required context
    auto &bestPairEntry = pairResults[candidate.objectName];
    EdgeMatrixEntry bestMatEntry = bestPairEntry.matrixResult->getBestPathMatEntry();
    ReloPushBossDiagnostics::log_try_stage(
        "matrix_entry_loaded",
        candidate,
        "edges=" + std::to_string(bestMatEntry.edgesInfo.size()) +
            ",vertices=" + std::to_string(bestMatEntry.vertexChain.size()) +
            ",obs_relo=" + std::to_string(bestMatEntry.obsReloList.size()));

    std::vector<EdgePath> ObsReloPathList;
    std::unordered_map<std::string, ReloPush::State> ToUpdate;
    ReloPush::StatePathPtrList transitPaths;

    ReloPush::State firstAppGoal;

    std::unordered_map<std::string, ReloPush::State> obsReloUpdate;

    // Plan obs relocations (if two or more). Transit paths between obsRelo
    for (size_t obs = 1; obs < bestMatEntry.obsReloList.size(); obs++)
    {
        auto pivotObj = bestMatEntry.vertexChain[obs];
        auto pivotObjInfo = pivotObj.toObjectInfo();
        auto this_pair = bestMatEntry.obsReloList[obs - 1];
        auto next_pair = bestMatEntry.obsReloList[obs];

        auto fromState = this_pair.second; // end of prev obs relo (pivot)
        auto toState = next_pair.first;    // start of next obs relo
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        auto toState_pre = find_pre_push(toState, planCtx.parameters.PrePush_dist);

        if (!planCtx.env_push.stateValid(fromState_pre))
        {
            ReloPushBossDiagnostics::log_try_stage(
                "fail_obs_relo_start_invalid", candidate,
                pivotObjInfo.name + " from=" +
                    ReloPushBossDiagnostics::state_to_string(fromState_pre));
            return false;
        }

        auto res = attemptObsRelocation(planCtx, fromState_pre, toState_pre, this_pair.first, this_pair.second,
                                        pairResults, candidate, ObsReloPathList, ToUpdate, pivotObjInfo, fromState);
        if (res->validity != PlanValidity::success)
        {
            ReloPushBossDiagnostics::log_try_stage(
                "fail_obs_relo_plan", candidate, pivotObjInfo.name);
            return false;
        }

        // Obs Relo ok
        obsReloUpdate.insert(std::make_pair(pivotObjInfo.name, this_pair.second));
    }

    // Plan from last relocation to final push (if any)
    // 1. Robot to first obsRelo start [robot -> firstObsStart]
    // 2. Last obsRelo start to pushing start [lastObsStart -> FirstWaypoint]
    if (!bestMatEntry.obsReloList.empty())
    {

        auto last_pair = bestMatEntry.obsReloList.back();
        auto first_pair = bestMatEntry.obsReloList.front();

        auto fromState = last_pair.second;
        auto &best_obj = objects[candidate.objectName];
        auto final_approach_obs = ReloPush::State(best_obj.x, best_obj.y, best_obj.getOrientation(candidate.row));
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        auto toState_pre = bestPairEntry.matrixResult->getBestPathMatEntry().edgesInfo[0].paths[0]->getFirstWaypoint();

        if (planCtx.env_push.stateValid(fromState_pre).get_validity() == StateValidity::out_of_boundary)
        {
            ReloPushBossDiagnostics::log_try_stage(
                "fail_last_obs_relo_start_boundary", candidate,
                ReloPushBossDiagnostics::state_to_string(fromState_pre));
            return false;
        }

        planCtx.checkObsCount("\t2");

        auto res_lastobs = attemptObsRelocation(planCtx, fromState_pre, toState_pre, last_pair.first, last_pair.second,
                                                pairResults, candidate, ObsReloPathList, ToUpdate,
                                                bestMatEntry.vertexChain[bestMatEntry.vertexChain.size() - 2].toObjectInfo(),
                                                fromState); // last obs relo to first wpt
        if (res_lastobs->validity != PlanValidity::success)
        {
            ReloPushBossDiagnostics::log_try_stage(
                "fail_last_obs_relo_plan", candidate);
            return false;
        }

        // Obs Relo ok
        obsReloUpdate.insert(std::make_pair(bestMatEntry.vertexChain[bestMatEntry.vertexChain.size() - 2].name, last_pair.second));

        // update first approach goal
        firstAppGoal = find_pre_push(first_pair.first, planCtx.parameters.PrePush_dist); // todo: start of first pair
    }

    // no Obstacle Relocation
    else
    {
        // auto obj_info = objects[candidate.objectName];
        // ReloPush::State obj_start = obj_info.getPushingPose(candidate.row);
        // ReloPush::State obj_start_pre = find_pre_push(obj_start,planCtx.parameters.PrePush_dist);

        firstAppGoal = bestMatEntry.edgesInfo[0].paths.at(0)->getFirstWaypoint();
    }

    // Plan approach (transit)
    // planCtx.checkObsCount("\t3");
    // auto res_app = planHybridAstar(robot, obj_start_pre, planCtx, true);

    auto res_app = planHybridAstar(robot, firstAppGoal, planCtx_backup, true); // before any obs relocation
    if (res_app->validity != PlanValidity::success)
    {
        /*
                // try once more with more margins
                auto onceMoreRobot = find_pre_push(robot,0.1);
                auto onceMoreGoal = find_pre_push(firstAppGoal,0.1);

                auto res_app_om = planHybridAstar(onceMoreRobot, onceMoreGoal,planCtx,true);
                {
                    if(res_app_om->validity != PlanValidity::success)
                    {
                        // restore obstacles
                        planCtx.updateObs(obs_backup);
                        return false;
                    }
                    else
                    {
                        res_app = res_app_om;
                    }
                }
        */
        planCtx.updateObs(obs_backup);
        ReloPushBossDiagnostics::log_try_stage(
            "fail_first_approach", candidate,
            "robot=" + ReloPushBossDiagnostics::state_to_string(robot) +
                ",goal=" + ReloPushBossDiagnostics::state_to_string(firstAppGoal));
        return false;
    }
    // transitPaths.push_back(res_app->getPathPtr(true));
    ReloPush::StatePathPtr firstApp = res_app->getPathPtr(true); // store it to allocation if evertying is fine

    // planCtx.checkObsCount("\t4");
    //  Plan transit paths between edges if needed. (if next edge has pre-relocation, it will likely to have different start)
    if (bestMatEntry.edgesInfo.size() > 1)
    {
        transitPaths.clear();
        for (size_t n = 1; n < bestMatEntry.edgesInfo.size(); n++)
        {
            auto last_goal = bestMatEntry.edgesInfo[n - 1].paths.back()->getLastWaypoint();
            auto this_start = bestMatEntry.edgesInfo[n].paths.front()->getFirstWaypoint();

            // inter-vertex movement (object moved to next vertex)
            auto prev_vertex = bestMatEntry.edgesInfo[n - 1].srcVertexData;
            auto next_vertex = bestMatEntry.edgesInfo[n - 1].sinkVertexData;

            auto prev_obj = prev_vertex.toObjectInfo();
            auto next_obj = next_vertex.toObjectInfo();

            // temporary change in obstacles
            auto moved_obj = next_obj;
            moved_obj.name = prev_obj.name;
            planCtx.removeObs(prev_obj);
            planCtx.addObs(moved_obj);

            auto res = planHybridAstar(last_goal, this_start, planCtx, true);
            if (res->validity != PlanValidity::success)
            {
                // restore obstacles
                planCtx.updateObs(obs_backup);
                ReloPushBossDiagnostics::log_try_stage(
                    "fail_edge_transit", candidate,
                    "edge_index=" + std::to_string(n));
                return false;
            }

            // put back
            planCtx.removeObs(moved_obj);
            planCtx.addObs(prev_obj);

            transitPaths.push_back(res->getPathPtr(true));
        }
    }
    // planCtx.checkObsCount("\t5");

    // Build FinalAllocation
    outAllocation.object = objects[candidate.objectName];
    outAllocation.goal = goals[candidate.goalName];
    outAllocation.cost = candidate.cost;
    outAllocation.row = candidate.row;
    outAllocation.col = candidate.col;
    outAllocation.vertexChain = bestMatEntry.vertexChain;
    outAllocation.edgeTransitPaths = transitPaths;
    outAllocation.startPose = ReloPush::State(outAllocation.object.x, outAllocation.object.y,
                                              outAllocation.object.getOrientation(outAllocation.row));
    outAllocation.goalPose = ReloPush::State(outAllocation.goal.x, outAllocation.goal.y,
                                             outAllocation.goal.getOrientation(outAllocation.col));
    outAllocation.paths = pairResults[candidate.objectName].matrixResult->getBestPathMatEntry().edgesInfo;
    outAllocation.obsReloPaths = std::make_shared<std::vector<EdgePath>>(ObsReloPathList);
    outAllocation.obsReloUpdate = obsReloUpdate;
    outAllocation.snapshot = planCtx;
    outAllocation.firstApproachPath = firstApp;
    // planCtx.checkObsCount("\t6");

    // Commit the ToUpdate states (update object positions)
    for (const auto &pair : ToUpdate)
    {
        objects[pair.first].x = pair.second.x;
        objects[pair.first].y = pair.second.y;
        // If you need to update orientation or other fields, do it here.
    }

    planCtx.checkObsCount("\t7");
    ReloPushBossDiagnostics::log_try_stage(
        "success", candidate,
        "first_app_wp=" + std::to_string(ReloPushBossDiagnostics::path_size(firstApp)) +
            ",obs_paths=" + std::to_string(ObsReloPathList.size()) +
            ",edge_transit=" + std::to_string(transitPaths.size()));

    return true;
}

// Main DFS function
// todo: gather all parameters into one object
bool performAllocationsDFS(
    const WorkspaceBoundary &boundary,
    ObjectMap objects,
    GoalMap goals,
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs,
    GoalMap delivered_objs,
    ReloPush::State robot,
    std::vector<FinalAllocation> &finalSequence,
    bool use_opt,
    bool no_init_guess,
    const std::chrono::time_point<std::chrono::high_resolution_clock> time_start,
    int depth)
{
    // ---- Base case: all objects delivered ----
    if (objGoalPairs.empty())
    {
        return true;
    }

    // handle timeout
    auto time_now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start);

    if (duration.count() > 180000) // 180 seconds
    {
        // timeout
        return false;
    }

    printCurrentState(finalSequence, objGoalPairs, delivered_objs);

    // ---- Build the graph for the current subproblem ----
    Graph g;
    initGraph(g, objects, goals);

    PlanningParameters params;
    params.boundary = boundary;
    PlanningContext planCtx(params, objects, delivered_objs, use_opt, no_init_guess);

    ReloPushBossDiagnostics::set_current_depth(depth);
    buildAllEdges(g, planCtx);
    ReloPushBossDiagnostics::log_search_state(
        depth, objects, goals, objGoalPairs, delivered_objs, robot, g);

    // saveGraphState(g, std::string(CMAKE_SOURCE_DIR) + "/gvis4.txt");

    // ---- Compute all pairwise assignments/costs ----
    auto pairResults = computeMatrixPairs(g, objGoalPairs, planCtx);
    ReloPushBossDiagnostics::log_pair_results(depth, pairResults);

    planCtx.checkObsCount("0");

    // ---- Iterate through sorted candidate pairs ----
    while (true)
    {
        planCtx.checkObsCount("1");
        // Get sorted global candidate list
        auto sortedCandidates = getSortedPairCandidates(pairResults);
        ReloPushBossDiagnostics::log_candidate_list(depth, sortedCandidates);
        // No more candidates? No solution at this recursion
        if (sortedCandidates.empty())
            return false;

        // Pick the best
        auto candidate = sortedCandidates.front();
        ReloPushBossDiagnostics::log_try_candidate(depth, candidate);
        if (candidate.row == -1 || candidate.col == -1 || candidate.cost == std::numeric_limits<double>::infinity())
            return false;

        // Backup for backtracking
        auto old_objects = objects;
        auto old_goals = goals;
        auto old_objGoalPairs = objGoalPairs;
        auto old_delivered_objs = delivered_objs;
        auto old_robot = robot;

        FinalAllocation allocation;
        planCtx.checkObsCount("before Alloc");
        bool ok = tryAllocation(candidate, pairResults, planCtx, objects, goals, objGoalPairs, delivered_objs, robot, allocation);
        ReloPushBossDiagnostics::log_try_result(
            depth, candidate, ok, ok ? &allocation : nullptr);
        if (!ok)
        {
            ReloPushBossDiagnostics::log_dfs_child_result(
                depth, candidate, false, false, finalSequence);
        }
        planCtx.checkObsCount("After Alloc " + std::to_string(depth));

        if (ok)
        {
            // for debug only
            // std::string rearr_obj = candidate.objectName;
            // std::cout << "ro: " << rearr_obj << std::endl;

            planCtx.checkObsCount("Alloc ok");
            delivered_objs[candidate.objectName] = goals[candidate.goalName];
            objects.erase(candidate.objectName);
            goals.erase(candidate.goalName);
            objGoalPairs.erase(candidate.objectName);
            finalSequence.push_back(allocation);
            ReloPushBossDiagnostics::log_commit(depth, allocation);

            // update robot pose
            robot = find_pre_push(allocation.goalPose, planCtx.parameters.PrePush_dist);

            // update obsRelo if any
            for (auto &kv : allocation.obsReloUpdate)
            {
                auto objNameObs = kv.first;
                auto &objNewPose = kv.second;

                objects.updateObjectPosition(objNameObs, objNewPose);
            }

            bool child_success = performAllocationsDFS(boundary, objects, goals, objGoalPairs, delivered_objs, robot, finalSequence, use_opt, no_init_guess, time_start, depth + 1);
            ReloPushBossDiagnostics::log_dfs_child_result(
                depth, candidate, true, child_success, finalSequence);
            if (child_success)
            {
                return true;
            }

            planCtx.checkObsCount("Alloc ng");
            // Backtrack
            ReloPushBossDiagnostics::log_backtrack(depth, allocation);
            finalSequence.pop_back();
            objects = old_objects;
            goals = old_goals;
            objGoalPairs = old_objGoalPairs;
            delivered_objs = old_delivered_objs;
            robot = old_robot;
            // planCtx.checkObsCount("2");
        }
        // planCtx.checkObsCount("2-3 " + std::to_string(depth));
        //  If fail: invalidate this candidate and try next best
        //  Mark cost as infinity in the matrix and remove from sortedEntries
        auto &matrixRes = pairResults[candidate.objectName].matrixResult;
        matrixRes->costMat(candidate.row, candidate.col) = std::numeric_limits<double>::infinity();
        auto &sorted = matrixRes->sortedEntries;
        sorted.erase(std::remove_if(sorted.begin(), sorted.end(),
                                    [&](const RowColCost &rcc)
                                    { return rcc.row == candidate.row && rcc.col == candidate.col; }),
                     sorted.end());
        ReloPushBossDiagnostics::log_candidate_invalidated(
            depth, candidate, ok ? "child_failed" : "allocation_failed");

        // planCtx.checkObsCount("3");
    }
}

// planning loop without DSP
bool performAllocations(
    const WorkspaceBoundary &boundary,
    ObjectMap objects,
    GoalMap goals,
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs,
    GoalMap delivered_objs,
    ReloPush::State robot,
    std::vector<FinalAllocation> &finalSequence,
    bool use_opt,
    bool no_init_guess,
    const std::chrono::time_point<std::chrono::high_resolution_clock> time_start)
{
    // ---- Base case: all objects delivered ----
    if (objGoalPairs.empty())
    {
        return true;
    }

    printCurrentState(finalSequence, objGoalPairs, delivered_objs);

    while (!objGoalPairs.empty())
    {
        // handle timeout
        auto time_now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_now - time_start);

        if (duration.count() > 180000) // 180 seconds
        {
            // timeout
            return false;
        }

        std::cout << std::flush;

        std::cout << "\n============================\n"
                  << "Remaining pairs: " << objGoalPairs.size() << "\n";

        // (a) Build the graph from scratch
        Graph g;
        initGraph(g, objects, goals);

        // (b) Setup PlanningParameters and context
        PlanningParameters params;
        params.boundary = boundary;
        PlanningContext planCtx(params, objects, delivered_objs, use_opt, false); // todo: add no_init_guess

        // (d) Build edges
        buildAllEdges(g, planCtx);
        const int diag_depth = static_cast<int>(finalSequence.size());
        ReloPushBossDiagnostics::log_search_state(
            diag_depth, objects, goals, objGoalPairs, delivered_objs, robot, g);

        // (e) Compute cost matrices for all remaining pairs
        auto pairResults = computeMatrixPairs(g, objGoalPairs, planCtx);
        ReloPushBossDiagnostics::log_pair_results(diag_depth, pairResults);

        // We'll store info about the best pick
        LowestCostInfo bestPick;
        std::vector<EdgePath> ObsReloPathList;
        std::unordered_map<std::string, ReloPush::State> ToUpdate;
        EdgeMatrixEntry bestMatEntry;
        ReloPush::StatePathPtrList transitPaths;

        // Take a snapshot of the planning context (for FinalAllocation)
        PlanningContext ctxSnapshot(planCtx);

        // Start searching for a feasible solution
        bool isFeasible = false;
        ReloPush::StatePathPtr firstApp = nullptr;
        while (!isFeasible)
        {
            isFeasible = findFeasibleAllocation(pairResults, objGoalPairs,
                                                planCtx, ObsReloPathList,
                                                bestPick, ToUpdate, bestPick.objectName, objects, bestMatEntry, transitPaths, robot, firstApp);
            ReloPushBossDiagnostics::log_try_candidate(diag_depth, bestPick);
            ReloPushBossDiagnostics::log_try_result(diag_depth, bestPick, isFeasible);
            if (isFeasible)
                break;

            // If no feasible solution, break or handle failure
            if (bestPick.row == -1 || bestPick.cost == std::numeric_limits<double>::infinity())
            {
                std::cerr << "Failure: no feasible solution found for any pair.\n";
                return false;
            }
            else
            {
                // update matrix and find next best
                // approach failed. Adjust cost matrix for re-planning.
                pairResults[bestPick.objectName].matrixResult->sortedEntries.erase(pairResults[bestPick.objectName].matrixResult->sortedEntries.begin()); // pop the first
                pairResults[bestPick.objectName].matrixResult->costMat(bestPick.row, bestPick.col) = std::numeric_limits<double>::infinity();             // mark inf on cost matrix
                ReloPushBossDiagnostics::log_candidate_invalidated(diag_depth, bestPick);
                continue;                                                                                                                                 // try other options
            }
        }

        // If we found a valid bestPick, commit it:
        //  Update object positions that got relocated
        for (const auto &pair : ToUpdate)
        {
            std::cout << "Relocated: " << pair.first
                      << " => " << pair.second << std::endl;

            objects[pair.first].x = pair.second.x;
            objects[pair.first].y = pair.second.y;
            // orientation if needed ...
        }

        // Mark the chosen object as delivered
        delivered_objs[bestPick.objectName] = goals[bestPick.goalName];

        // Print the best pair
        std::cout << "BEST PAIR => " << bestPick.objectName
                  << " -> " << bestPick.goalName
                  << ", cost=" << bestPick.cost
                  << ", row=" << bestPick.row
                  << ", col=" << bestPick.col << "\n";

        // Build the FinalAllocation entry
        FinalAllocation chosen;
        chosen.object = objects[bestPick.objectName];
        chosen.goal = goals[bestPick.goalName];
        chosen.cost = bestPick.cost;
        chosen.row = bestPick.row;
        chosen.col = bestPick.col;
        chosen.vertexChain = bestMatEntry.vertexChain;
        chosen.edgeTransitPaths = transitPaths;

        chosen.startPose = ReloPush::State(chosen.object.x, chosen.object.y,
                                           chosen.object.getOrientation(chosen.row));
        chosen.goalPose = ReloPush::State(chosen.goal.x, chosen.goal.y,
                                          chosen.goal.getOrientation(chosen.col));

        // chosen.paths = pairResults[bestPick.objectName].getBestPath().edgesInfo;
        chosen.paths = pairResults[bestPick.objectName].matrixResult->getBestPathMatEntry().edgesInfo;
        chosen.obsReloPaths = std::make_shared<std::vector<EdgePath>>(ObsReloPathList);
        // chosen.obsReloUpdate = obsReloUpdate;
        chosen.snapshot = ctxSnapshot;
        chosen.firstApproachPath = firstApp;

        finalSequence.push_back(chosen);
        ReloPushBossDiagnostics::log_commit(diag_depth, chosen);

        // (h) Remove the chosen pair so we don’t pick it again
        objGoalPairs.erase(bestPick.objectName);
        objects.erase(bestPick.objectName);
        goals.erase(bestPick.goalName);

        // update robot pose
        robot = find_pre_push(chosen.goalPose, planCtx.parameters.PrePush_dist);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Helper Function 5: Print the final sequence
// ---------------------------------------------------------------------------
void printFinalSequence(const std::vector<FinalAllocation> &finalSequence)
{
    std::cout << "\nFinal sequence of chosen tasks:\n";
    for (auto &fa : finalSequence)
    {
        std::cout << "Object = " << fa.object.name
                  << ", Goal = " << fa.goal.name
                  << ", cost = " << fa.cost << "\n"
                  << "  start yaw = " << fa.startPose.yaw
                  << ", goal yaw = " << fa.goalPose.yaw;

        for (auto &it : fa.paths)
        {
            if (it.preRelo.used)
            {
                std::cout << " Pre-Relo: (" << it.preRelo.xRelocated_object << ", " << it.preRelo.yRelocated_object << ")";
            }
        }

        if (fa.obsReloPaths->size() > 0)
        {
            std::cout << ", ObsRelo steps: " << fa.obsReloPaths->size() << "\n";
        }
        else
        {
            std::cout << std::endl;
        }

        bool print_trajectory = false;
        if (print_trajectory)
        {
            if (fa.obsReloPaths->size() > 0)
                std::cout << "Obs-Relo" << std::endl;
            for (auto &it : *fa.obsReloPaths)
                it.print();

            std::cout << "Path" << std::endl;
            for (size_t n = 0; n < fa.paths.size(); n++)
            {
                if (n != 0)
                {
                    std::cout << "transit " << fa.edgeTransitPaths[n - 1]->size() << std::endl;
                }

                fa.paths[n].printPath();
                std::cout << std::endl;
            }
        }
    }

    // calculate total cost
    double cost_sum = 0;
    for (auto &it : finalSequence)
        cost_sum += it.cost;

    std::cout << "Total Cost: " << cost_sum << std::endl;
}
