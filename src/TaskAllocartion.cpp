#include<TaskAllocation.hpp>

void PairCostResult::remove_top(void)
{
    // remove from cost matrix
    matrixResult.costMat(bestRow,bestCol) = std::numeric_limits<double>::infinity();
    // if any left
    if (!matrixResult.sortedEntries.empty()) {
        // remove from sortedEntries
        matrixResult.sortedEntries.erase(matrixResult.sortedEntries.begin());
        // update next best
        bestCost = matrixResult.sortedEntries[0].cost;
        bestRow = matrixResult.sortedEntries[0].row;
        bestCol = matrixResult.sortedEntries[0].col;
    }
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
        MatrixResult matrixRes = computeCostMatrix(g, objVerts, goalVerts);

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
        pcr.matrixResult= matrixRes;

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

/**
 * @brief Finds the single lowest cost among all PairCostResult entries,
 *        returning its details (object, goal, row, col, cost, and index).
 *
 * @param results The vector of PairCostResult from computeAndSortAllPairs().
 * @return A LowestCostInfo with the absolute minimal cost found.
 *         If 'results' is empty, fields will be default/invalid.
 */
LowestCostInfo findAbsoluteLowestCost(const std::vector<PairCostResult> &results)
{
    LowestCostInfo best;
    best.cost = std::numeric_limits<double>::infinity();
    best.indexInArray = -1;
    best.row = -1;
    best.col = -1;

    for (size_t i = 0; i < results.size(); ++i)
    {
        const auto &p = results[i];
        double c = p.bestCost;

        if (c < best.cost)
        {
            best.cost         = c;
            best.indexInArray = static_cast<int>(i);
            best.objectName   = p.objectName;
            best.goalName     = p.goalName;
            best.row          = p.bestRow;
            best.col          = p.bestCol;
        }
    }

    return best;
}



