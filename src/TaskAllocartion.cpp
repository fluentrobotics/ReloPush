#include<TaskAllocation.hpp>

EdgeMatrixEntry PairCostResult::getBestPath()
{
    //return matrixResult->pathMat->at(bestRow)[bestCol];
    return matrixResult->getBestPathMatEntry();
}

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

void PathsToSinglePath(std::vector<EdgeData>& paths, std::vector<size_t>& path_sizes,
                       ReloPush::StatePath& out_path, double interpolation_resolution)
{
    for(auto& it : paths)
    {
        for(auto it2 : it.paths)
        {
            ReloPush::StatePathPtr statePath;
            // Check if the variant holds a StatePathPtr
            if (std::holds_alternative<ReloPush::StatePathPtr>(it2->path))
            {
                statePath = std::get<ReloPush::StatePathPtr>(it2->path);

            }
            // If needed, handle reloDubinsPath here (currently ignored)
            else if(std::holds_alternative<reloDubinsPath>(it2->path))
            {
                auto dubinsPath = std::get<reloDubinsPath>(it2->path);
                statePath = dubinsPath.interpolate(interpolation_resolution); // todo: parse map resolution
            }

            // fill out_path
            for(auto& p : *statePath)
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
    for(auto it : *paths)
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

double FinalAllocation::getPushingLength(void) const
{
    double obsPush = 0;
    double taskPush = 0;

    // sum obs push
    for(auto& it : *obsReloPaths)
    {
        if(it.is_pushing)
            obsPush += it.getLength();
    }

    for(auto& it : paths) // for each edge
    {
        for(auto& it2 : it.paths) // for each path
        {
            if(it2->is_pushing)
                taskPush += it2->getLength();
        }
    }

    return obsPush + taskPush;
}

std::pair<ReloPush::StatePathPtr,std::vector<size_t>> FinalAllocation::toSinglePathPtr(double interpolation_resolution)
{
    ReloPush::StatePath obs_path(0);
    ReloPush::StatePath out_path(0);



    // add ObsRelo
    //for(size_t n=0; n<obsReloPaths->size(); n++)
    //{
        auto pathPtr = EdgePathListToSinglePath(obsReloPaths,0.2);
        obs_path.insert(obs_path.end(), pathPtr->begin(), pathPtr->end());
    //}

    std::vector<size_t> path_sizes ={obs_path.size()};
    PathsToSinglePath(paths, path_sizes,out_path, interpolation_resolution);

    ReloPush::StatePath combined;
    // Reserve space for performance (optional).
    combined.reserve(obs_path.size() + out_path.size());

    // Insert all elements from vec1 and then vec2.
    combined.insert(combined.end(), obs_path.begin(), obs_path.end());
    combined.insert(combined.end(), out_path.begin(), out_path.end());



    return std::make_pair(std::make_shared<ReloPush::StatePath>(combined), path_sizes);
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

ReloPush::StatePathPtr Find_ObsRelo(ObjectInfo& mo, PlanningContext& ctx, std::vector<EdgeData>& edgesInfo)
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


            auto obs = ctx.env_push.get_obs();
            // add path points as obstacles
            std::vector<ReloPush::State> pathObs;
            std::vector<size_t> path_sizes; // dummy
            PathsToSinglePath(edgesInfo,path_sizes,pathObs,ctx.parameters.obs_rad*2);
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
                        EdgeData ed = g[e];
                        // copy the entire EdgeData
                        //ed = g[e];
                        // Now copy all EdgePaths from g[e].paths
                        //std::vector<EdgePath> temp_list;
                        //for (auto &ep : g[e].paths)
                        //{
                            //auto epPtr = std::make_shared<EdgePath>(ep);
                        //    temp_list.push_back(*ep);
                        //}
                        //pair.edgeData.paths = std::make_shared<std::vector<EdgePath>>(temp_list);
                        edgesInfo.edgesInfo.push_back(std::move(ed));
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
                        auto pivotObj = ctx.mo_list[edgesInfo.edgesInfo[n].srcVertexData.name];
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
                    chain.push_back(edgesInfo.edgesInfo[0].srcVertexData);
                    for (auto &ed : edgesInfo.edgesInfo)
                    {
                        chain.push_back(ed.sinkVertexData);
                    }
                }

                // Build RowColCost
                RowColCost rcc;
                rcc.row  = static_cast<int>(i);
                rcc.col  = static_cast<int>(j);
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
        //pcr.bestCost     = bestCost;
        //pcr.bestRow      = bestRow;
        //pcr.bestCol      = bestCol;
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
        for (auto &rcc : mResPtr->sortedEntries) // todo: it is already sorted. only need to compare the first of each
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
                          ReloPush::State &fromObs,         // obstacle to remove
                          ReloPush::State &toObs,           // obstacle to add
                          PairResultsMap &pairResults,
                          const LowestCostInfo &bestPick,
                          std::vector<EdgePath> &ObsReloPathList,
                          std::unordered_map<std::string, ReloPush::State> &ToUpdate,
                          const std::string &pivotObjName,
                          const ReloPush::State &objNewState)
{
    // 1) Environment updates
    planCtx.removeObs(fromObs);
    planCtx.addObs(toObs);

    // 2) Attempt path planning (after obs relo)
    auto res = planHybridAstar(fromState_prepush, toState_prepush, planCtx, true);
    if (!res->success)
    {
        // revert environment changes
        planCtx.addObs(fromObs);
        planCtx.removeObs(toObs);

        return res;
    }

    // 3) If success, record the path
    ReloPush::StatePath pre_pair_path = {
        find_pre_push(fromObs, planCtx.parameters.PrePush_dist),
        fromState_prepush
    };
    // The first EdgePath is a trivial “push” from some pre-push position
    ObsReloPathList.push_back(EdgePath(/*isPrePush=*/true, std::make_shared<ReloPush::StatePath>(pre_pair_path)));
    // The second EdgePath is the actual planned path
    ObsReloPathList.push_back(EdgePath(/*isPrePush=*/false, res->getPathPtr(true)));

    // 4) Update the object’s new location
    ToUpdate[pivotObjName] = objNewState;

    return res;
}

// ---------------------------------------------------------------------------
// Helper Function 3: One iteration of picking the best pair and planning
// ---------------------------------------------------------------------------
bool findFeasibleAllocation(PairResultsMap &pairResults,
                            const std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                            PlanningContext &planCtx,
                            std::vector<EdgePath> &ObsReloPathList,
                            LowestCostInfo &bestPick,
                            std::unordered_map<std::string, ReloPush::State> &ToUpdate,
                            std::string &failedObjectName, ObjectMap objects, EdgeMatrixEntry& bestMatEntry, ReloPush::StatePathPtrList& transitPaths)
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
        failedObjectName = bestPick.objectName;  // Let caller handle
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

        auto fromState     = prev_pair.second;
        auto toState       = next_pair.first;
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        auto toState_pre   = find_pre_push(toState, planCtx.parameters.PrePush_dist);

        // check start valid
        auto sv = planCtx.env_push.stateValid(fromState_pre);
        if(!sv)
            return false;

        // Attempt relocation
        auto res = attemptObsRelocation(planCtx,
                                            fromState_pre, toState_pre,
                                            prev_pair.first,  // obs to remove
                                            prev_pair.second, // obs to add
                                            pairResults, bestPick,
                                            ObsReloPathList,
                                            ToUpdate,
                                            pivotObj.name,
                                            fromState);
        if (!res->success)
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
        auto last_pair  = bestMatEntry.obsReloList.back();
        auto fromState  = last_pair.second;
        auto best_obj   = objects[bestPick.objectName]; // object to deliver
        // final approach
        auto final_approach_obs = ReloPush::State(best_obj.x,
                                              best_obj.y,
                                              best_obj.getOrientation(bestPick.row));
        auto fromState_pre = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
        //auto toState_pre   = find_pre_push(final_approach_obs, planCtx.parameters.PrePush_dist);
        auto toState_pre = bestPairEntry.matrixResult->getBestPathMatEntry().edgesInfo[0].paths[0]->getFirstWaypoint(); // picked by sorted entries
        // check start valid
        auto sv = planCtx.env_push.stateValid(fromState_pre);
        if(sv.get_validity()==StateValidity::out_of_boundary)
            return false;

        auto res = attemptObsRelocation(planCtx,
                                            fromState_pre, toState_pre,
                                            last_pair.first,
                                            last_pair.second,
                                            pairResults, bestPick,
                                            ObsReloPathList,
                                            ToUpdate,
                                            bestMatEntry.vertexChain[bestMatEntry.vertexChain.size() - 2].name,
                                            fromState);
        if (!res->success)
        {
            failedObjectName = bestPick.objectName;
            return false;
        }
    }

    // For multiple edges, check and plan transit between the two
    if(bestMatEntry.edgesInfo.size()>1)
    {
        transitPaths.clear();
        for(size_t n=1; n<bestMatEntry.edgesInfo.size(); n++)
        {
            auto last_goal = bestMatEntry.edgesInfo[n-1].paths.back()->getLastWaypoint();
            auto this_start = bestMatEntry.edgesInfo[n].paths.front()->getFirstWaypoint();

            auto res = planHybridAstar(last_goal,this_start,planCtx,true);

            if(!res->success)
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

// ---------------------------------------------------------------------------
// Helper Function 4: The main planning/allocation loop
// ---------------------------------------------------------------------------
bool performAllocations(const WorkspaceBoundary &boundary,
                        std::unordered_map<std::string, ObjectInfo> &objects,
                        std::unordered_map<std::string, GoalInfo> &goals,
                        std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                        std::vector<FinalAllocation> &finalSequence,
                        bool& use_opt)
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
        PlanningContext planCtx(params, objects, delivered_objs, use_opt);

        // (d) Build edges
        buildAllEdges(g, planCtx);

        // (e) Compute cost matrices for all remaining pairs
        auto pairResults = computeMatrixPairs(g, objGoalPairs, planCtx);

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
            isFeasible =  findFeasibleAllocation(pairResults, objGoalPairs,
                                                     planCtx, ObsReloPathList,
                                                         bestPick, ToUpdate, /*out*/bestPick.objectName, objects, bestMatEntry, transitPaths);
            if(isFeasible)
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
                pairResults[bestPick.objectName].matrixResult->sortedEntries.erase(pairResults[bestPick.objectName].matrixResult->sortedEntries.begin()); //pop the first
                pairResults[bestPick.objectName].matrixResult->costMat(bestPick.row, bestPick.col) = std::numeric_limits<double>::infinity(); // mark inf on cost matrix
                continue; // try other options
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
        chosen.goal   = goals[bestPick.goalName];
        chosen.cost   = bestPick.cost;
        chosen.row    = bestPick.row;
        chosen.col    = bestPick.col;
        chosen.vertexChain = bestMatEntry.vertexChain;
        chosen.transitPaths = transitPaths;

        chosen.startPose = ReloPush::State(chosen.object.x, chosen.object.y,
                                           chosen.object.getOrientation(chosen.row));
        chosen.goalPose  = ReloPush::State(chosen.goal.x, chosen.goal.y,
                                          chosen.goal.getOrientation(chosen.col));

        //chosen.paths = pairResults[bestPick.objectName].getBestPath().edgesInfo;
        chosen.paths = pairResults[bestPick.objectName].matrixResult->getBestPathMatEntry().edgesInfo;
        chosen.obsReloPaths = std::make_shared<std::vector<EdgePath>>(ObsReloPathList);
        chosen.snapshot     = ctxSnapshot;

        finalSequence.push_back(chosen);

        // (h) Remove the chosen pair so we don’t pick it again
        objGoalPairs.erase(bestPick.objectName);
        objects.erase(bestPick.objectName);
        goals.erase(bestPick.goalName);
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

        for(auto& it : fa.paths)
        {
            if(it.preRelo.used)
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
        if(print_trajectory)
        {
            if(fa.obsReloPaths->size() >0)
                std::cout << "Obs-Relo" << std::endl;
            for(auto& it : *fa.obsReloPaths)
                it.print();

            std::cout << "Path" << std::endl;
            for(size_t n=0; n<fa.paths.size(); n++)
            {
                if(n!=0)
                {
                    std::cout << "transit " << fa.transitPaths[n-1]->size() << std::endl;
                }

                fa.paths[n].printPath();
                std::cout << std::endl;
            }
        }
    }

    // calculate total cost
    double cost_sum = 0;
    for(auto& it : finalSequence)
        cost_sum += it.cost;

    std::cout << "Total Cost: " << cost_sum << std::endl;

}
