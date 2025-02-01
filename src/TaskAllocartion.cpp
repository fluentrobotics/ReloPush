#include<TaskAllocation.hpp>

EdgeMatrixEntry PairCostResult::getBestPath()
{
    return matrixResult->pathMat->at(bestRow)[bestCol];
}

void PairCostResult::remove_top(void)
{
    // remove from cost matrix
    matrixResult->costMat(bestRow,bestCol) = std::numeric_limits<double>::infinity();
    // if any left
    if (!matrixResult->sortedEntries.empty()) {
        // remove from sortedEntries
        matrixResult->sortedEntries.erase(matrixResult->sortedEntries.begin());
        // update next best
        bestCost = matrixResult->sortedEntries[0].cost;
        bestRow = matrixResult->sortedEntries[0].row;
        bestCol = matrixResult->sortedEntries[0].col;
    }
}

void PathsToSinglePath(std::vector<EdgeDataPathPair>& paths, ReloPush::StatePath& out_path, double interpolation_resolution)
{
    for(auto& it : paths)
    {
        for(auto it2 : *it.edgePathList)
        {
            ReloPush::StatePathPtr statePath;
            // Check if the variant holds a StatePathPtr
            if (std::holds_alternative<ReloPush::StatePathPtr>(it2.path))
            {
                statePath = std::get<ReloPush::StatePathPtr>(it2.path);

            }
            // If needed, handle reloDubinsPath here (currently ignored)
            else if(std::holds_alternative<reloDubinsPath>(it2.path))
            {
                auto dubinsPath = std::get<reloDubinsPath>(it2.path);
                statePath = dubinsPath.interpolate(interpolation_resolution); // todo: parse map resolution
            }

            // fill out_path
            for(auto& p : *statePath)
            {
                out_path.push_back(p);
            }
        }
    }
}

ReloPush::StatePathPtr EdgePathListToSinglePath(EdgePathList paths, double resolution)
{
    ReloPush::StatePath out_path;
    for(auto& it : *paths)
    {
        ReloPush::StatePathPtr statePath;
        // Check if the variant holds a StatePathPtr
        if (std::holds_alternative<ReloPush::StatePathPtr>(it.path))
        {
            statePath = std::get<ReloPush::StatePathPtr>(it.path);
        }
        // If needed, handle reloDubinsPath here (currently ignored)
        else if(std::holds_alternative<reloDubinsPath>(it.path))
        {
            auto dubinsPath = std::get<reloDubinsPath>(it.path);
            statePath = dubinsPath.interpolate(resolution); // todo: parse map resolution
        }

        // fill out_path
        for(auto& p : *statePath)
        {
            out_path.push_back(p);
        }
    }

    return std::make_shared<ReloPush::StatePath>(out_path);
}


ReloPush::StatePathPtr FinalAllocation::toSinglePathPtr(double interpolation_resolution)
{
    ReloPush::StatePath obs_path(0);
    ReloPush::StatePath out_path(0);

    // add ObsRelo
    //for(size_t n=0; n<obsReloPaths->size(); n++)
    //{
        auto pathPtr = EdgePathListToSinglePath(obsReloPaths,0.2);
        obs_path.insert(obs_path.end(), pathPtr->begin(), pathPtr->end());
    //}

    PathsToSinglePath(paths,out_path, interpolation_resolution);

    ReloPush::StatePath combined;
    // Reserve space for performance (optional).
    combined.reserve(obs_path.size() + out_path.size());

    // Insert all elements from vec1 and then vec2.
    combined.insert(combined.end(), obs_path.begin(), obs_path.end());
    combined.insert(combined.end(), out_path.begin(), out_path.end());

    return std::make_shared<ReloPush::StatePath>(combined);
}


std::vector<ReloPush::State> FinalTaskSequence::to_StateList(void)
{
    std::vector<ReloPush::State> out_list(task_sequence.size());
    
    for(size_t n=0; n<task_sequence.size(); n++)
        out_list[n] = ReloPush::State(task_sequence[n].goal.x, task_sequence[n].goal.y, task_sequence[n].goal.nominalOrientation);

    return out_list;
}

/**
 * @brief Scans an Eigen::MatrixXd for its minimal value (if any).
 *        Returns (value, row, col). If matrix is empty, returns +inf.
 */
MatrixMinEntry findMatrixMin(const Eigen::MatrixXd &mat)
{
    MatrixMinEntry result;
    result.value = std::numeric_limits<double>::infinity();
    result.row   = -1;
    result.col   = -1;

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
                result.row   = r;
                result.col   = c;
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
            return a.cost < b.cost;
        }
        );

    // Build final result
    MatrixResult result;
    result.costMat       = costMatrix;
    result.sortedEntries = rowColList;

    return result;
}
*/

void sortByDistance(std::vector<ReloPush::State>& goals, const ReloPush::State& start) {
    std::sort(goals.begin(), goals.end(), [&start](const ReloPush::State& a, const ReloPush::State& b) {
        return StateDistance(start, a) < StateDistance(start, b);
    });
}

ReloPush::StatePathPtr Find_ObsRelo(ObjectInfo& mo, PlanningContext& ctx, std::vector<EdgeDataPathPair>& edgesInfo)
{
    auto init_pusing_poses = mo.getPushingPoses();

    std::vector<ReloPush::State> found_candidates(0);

    ReloPush::State objectPos(init_pusing_poses[0].x,init_pusing_poses[0].y,init_pusing_poses[0].yaw);
    for (const auto& pp : init_pusing_poses) {
        // Direction is represented as a pair of (dx, dy)
        double dx = cosf(pp.yaw) * ctx.parameters.map_resolution;
        double dy = sinf(pp.yaw) * ctx.parameters.map_resolution; // unit vector

        bool out_of_boundary = false;
        // Check positions along this direction
        ReloPush::State obsrelo_candidate = pp;
        while(!out_of_boundary)
        {
            obsrelo_candidate.x += dx;
            obsrelo_candidate.y += dy;

            //auto validity = env.stateValid(obsrelo_candidate,Constants::carWidth,2*Constants::obsRadius);
            StateValidity validity = StateValidity::valid;


            auto obs = ctx.env.get_obs();
            // add path points as obstacles
            std::vector<ReloPush::State> pathObs;
            PathsToSinglePath(edgesInfo,pathObs,ctx.parameters.obs_rad*2);
            obs.insert(pathObs.begin(), pathObs.end());


            for(auto& it: obs)
            {
                if(StateDistance(it,obsrelo_candidate)<Constants::obsRadius*2 + Constants::LF_nonpush + Constants::LB + 0.05)
                {
                    validity = StateValidity::collision;
                    break;
                }
                else if(obsrelo_candidate.x < ctx.parameters.boundary.xMin
                           || obsrelo_candidate.x > ctx.parameters.boundary.xMax
                           || obsrelo_candidate.y < ctx.parameters.boundary.yMin
                           || obsrelo_candidate.y > ctx.parameters.boundary.yMax)
                {
                    validity = StateValidity::out_of_boundary;
                    break;
                }
            }

            // out-of-bounday: finish with this vec
            if(validity == StateValidity::out_of_boundary)
            {
                out_of_boundary = true;
                break;
            }
            // obsrelo candidate found
            else if(validity == StateValidity::valid)
                break;
        }

        // not out-of-bounday: found a candidate
        if(!out_of_boundary)
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
                  return a.cost < b.cost;
              });

    // Fill in the final sorted list
    result.sortedEntries = rowColList;

    return result;
}
*/ //previous version

MatrixResultPtr computeCostMatrixWithPaths(
    const Graph &g,
    const std::vector<Vertex> &objectVerts,
    const std::vector<Vertex> &goalVerts,
    PlanningContext& ctx)
{
    // 1) Dimensions
    size_t Nobj  = objectVerts.size();
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
    size_t numV   = boost::num_vertices(g);

    // 4) For each object vertex i, run Dijkstra
    for (size_t i = 0; i < Nobj; ++i)
    {
        Vertex src = objectVerts[i];

        // Distances & predecessors
        std::vector<double> distMap(numV, std::numeric_limits<double>::infinity());
        std::vector<Vertex> predMap(numV, Graph::null_vertex());

        boost::dijkstra_shortest_paths(
            g, src,
            boost::distance_map(boost::make_iterator_property_map(distMap.begin(), indexMap))
                .predecessor_map(boost::make_iterator_property_map(predMap.begin(), indexMap))
                .weight_map(get(&EdgeData::weight, g))); // todo: skip unnecessary search

        // 4b) For each goal vertex j, reconstruct the path if finite
        for (size_t j = 0; j < Ngoal; ++j)
        {
            Vertex goalV = goalVerts[j];
            double d = distMap[indexMap[goalV]];

            result->costMat(i, j) = d;
            if (d < 1e9)  // finite
            {
                // Build RowColCost
                RowColCost rcc;
                rcc.row  = static_cast<int>(i);
                rcc.col  = static_cast<int>(j);
                rcc.cost = d;
                rowColList.push_back(rcc);

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
                    Edge e; bool hasEdge;
                    boost::tie(e, hasEdge) = boost::edge(p, cur, g);
                    if (hasEdge)
                    {
                        EdgeDataPathPair pair;
                        // copy the entire EdgeData
                        pair.edgeData = g[e];
                        // Now copy all EdgePaths from g[e].paths
                        std::vector<EdgePath> temp_list;
                        for (auto &ep : g[e].paths)
                        {
                            //auto epPtr = std::make_shared<EdgePath>(ep);
                            temp_list.push_back(ep);
                        }
                        pair.edgePathList = std::make_shared<std::vector<EdgePath>>(temp_list);
                        edgesInfo.edgesInfo.push_back(std::move(pair));
                    }
                    cur = p;
                }

                // The 'edgesInfo' is reversed (goal->...->src).
                // If you want them in forward order (src->...->goal), reverse:
                std::reverse(edgesInfo.edgesInfo.begin(), edgesInfo.edgesInfo.end());

                (*pathMatPtr)[i][j].obsReloList.clear();
                // handle multiple edges
                if(edgesInfo.edgesInfo.size()>1)
                {
                    for(size_t n=1; n<edgesInfo.edgesInfo.size(); n++)
                    {
                        // pivot object
                        auto pivotObj = ctx.mo_list[edgesInfo.edgesInfo[n].edgeData.srcVertexData.name];
                        // for each object
                        auto obsRelo_candidates = Find_ObsRelo(pivotObj, ctx, edgesInfo.edgesInfo);

                        // handle failure in finding ObsRelo
                        if(obsRelo_candidates->size()==0)
                        {
                            edgesInfo.edgesInfo.clear();
                            result->costMat(i, j) = std::numeric_limits<double>::infinity();
                            break;
                        }

                        // use first candidate
                        auto obsRelo_state = obsRelo_candidates->at(0);
                        ReloPush::State start_state = ReloPush::State(pivotObj.x,pivotObj.y,obsRelo_state.yaw);

                        double obs_push_d = ReloPush::StateDistance(start_state,obsRelo_state);
                        result->costMat(i, j) += obs_push_d;

                        //ReloPush::State start_pre_push = find_pre_push(start_state, params::pre_push_dist);
                        //ReloPush::State goal_pre_push = find_pre_push(obsRelo_state, params::pre_push_dist+params::pre_relo_pre_push_offset);

                        //(*pathMatPtr)[i][j].obsReloList.push_back(std::make_pair(start_pre_push, goal_pre_push));
                        edgesInfo.obsReloList.push_back(std::make_pair(start_state, obsRelo_state));
                    }

                }

                // Build vertexChain
                std::vector<VertexData> chain;
                if (!edgesInfo.edgesInfo.empty())
                {
                    chain.reserve(edgesInfo.edgesInfo.size() + 1); // optional performance
                    chain.push_back(edgesInfo.edgesInfo[0].edgeData.srcVertexData);
                    for (auto &pair : edgesInfo.edgesInfo)
                    {
                        chain.push_back(pair.edgeData.sinkVertexData);
                    }
                }

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
                  return a.cost < b.cost;
              });

    // 6) Fill in the final sorted list & pathMat
    result->sortedEntries = std::move(rowColList);
    result->pathMat       = pathMatPtr;

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
    const Graph &g, std::unordered_map<std::string, ObjectGoalPair> &pairs, PlanningContext& ctx)
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
        std::vector<Vertex> objVerts  = getObjectVertices(g, objName);
        std::vector<Vertex> goalVerts = getGoalVertices(g, goalName);

        // 2) Build the cost matrix (which also produces sorted entries + pathMat)
        MatrixResultPtr matrixRes = computeCostMatrixWithPaths(g, objVerts, goalVerts, ctx);

        // 3) The minimal cost entry is sortedEntries[0], unless the matrix is empty
        double bestCost  = std::numeric_limits<double>::infinity();
        int bestRow      = -1;
        int bestCol      = -1;

        if (!matrixRes->sortedEntries.empty())
        {
            const auto &top = matrixRes->sortedEntries[0];
            bestCost = top.cost;
            bestRow  = top.row;
            bestCol  = top.col;
        }

        // 4) Create a PairCostResult
        PairCostResult pcr;
        pcr.objectName   = objName;
        pcr.goalName     = goalName;
        pcr.bestCost     = bestCost;
        pcr.bestRow      = bestRow;
        pcr.bestCol      = bestCol;
        // store the entire MatrixResult in a shared_ptr
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
    best.row  = -1;
    best.col  = -1;

    // For each pair in the map (key=object name, value=PairCostResult)
    for (auto &kv : resultsMap)
    {
        const auto &pcr = kv.second;  // pcr is a PairCostResult
        auto mResPtr = pcr.matrixResult; // the MatrixResultPtr

        if (!mResPtr) // no matrix result? skip
            continue;

        // 1) Check all RowColCost in sortedEntries
        //    Each entry is (row, col, cost), sorted ascending, but we must
        //    look at them all because a "second best" in one pair might still
        //    be lower than the "best" in another pair.
        for (auto &rcc : mResPtr->sortedEntries)
        {
            if (rcc.cost < best.cost)
            {
                best.cost       = rcc.cost;
                best.row        = rcc.row;
                best.col        = rcc.col;
                best.objectName = pcr.objectName;
                best.goalName   = pcr.goalName;
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
