#ifndef TASKALLOCATION_HPP
#define TASKALLOCATION_HPP

#include <Eigen/Dense>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <GraphData.hpp>
#include <GraphBuilder.hpp>

#include <ObjectInfo.hpp>
#include <PlanHybridAstar.hpp>

#include <string>
#include <memory>

const auto ReloPushInf = std::numeric_limits<double>::infinity();



struct obj_goal_pair
{
    std::string objectName;
    std::string goalName;
};

/*
struct CostMatrixResult
{
    std::string objectName;    // which object
    std::string goalName;      // which goal
    Eigen::MatrixXd costMatrix; // entire cost matrix (objectVerts x goalVerts)

    double bestCost;           // minimal cost found in this matrix
    int bestRow;               // row index of that minimal cost
    int bestCol;               // col index of that minimal cost
};
*/

struct MatrixMinEntry
{
    double value;
    int row;
    int col;
};

struct RowColCost
{
    int row;
    int col;
    double cost;
    // A list of EdgeData for each edge on the shortest path.
    //std::vector<EdgeData> pathEdges;

    RowColCost()
    {
        row=-1;
        col=-1;
        cost=-1;
    }
};

using EdgePathList = std::shared_ptr<std::vector<EdgePath>>;

/*
struct EdgeDataPathPair
{
    EdgeData edgeData; // info on each edge
    //EdgePathList edgePathList; // trajectory todo: this seems to be a dubplicate of edgeData.paths;
};
*/

ReloPush::StatePathPtr EdgePathListToSinglePath(EdgePathList paths, double resolution);

void PathsToSinglePath(std::vector<EdgeData>& paths, ReloPush::StatePath& out_path, double interpolation_resolution);

using ObsReloPair = std::vector<std::pair<ReloPush::State,ReloPush::State>>;
struct EdgeMatrixEntry
{
    // A list of EdgeDataPathPair => each is (EdgeData + EdgePathList).
    std::vector<EdgeData> edgesInfo;

    // Now we add a chain of VertexData if we want the entire route's vertices.
    std::vector<VertexData> vertexChain;

    std::vector<std::pair<ReloPush::State,ReloPush::State>> obsReloList; // pair of start and goal for each obs relo

    EdgeMatrixEntry(){}
    EdgeMatrixEntry(std::vector<EdgeData> edgeData_in) : edgesInfo(edgeData_in)
    {}
};

using EdgeDataPathMatrix = std::vector<std::vector<EdgeMatrixEntry>>;
using EdgeDataPathMatrixPtr = std::shared_ptr<EdgeDataPathMatrix>;
using SortedEntryList = std::vector<RowColCost>;

struct MatrixResult
{
    Eigen::MatrixXd costMat; // The full cost matrix (row = object vertex, col = goal vertex).
    EdgeDataPathMatrixPtr pathMat; // A 2D array (size = [Nobj x Ngoal]) storing EdgePathPtr
    SortedEntryList sortedEntries; // A sorted list of (row, col, cost) in ascending order of cost.

    EdgeMatrixEntry getBestPathMatEntry()
    {
        return pathMat->at(sortedEntries[0].row)[sortedEntries[0].col];
    }
};
using MatrixResultPtr = std::shared_ptr<MatrixResult>;

class PairCostResult
{
public:
    std::string objectName;
    std::string goalName;

    //double bestCost;
    //int bestRow;
    //int bestCol;

    // The entire MatrixResult, which has costMat + sortedEntries
    MatrixResultPtr matrixResult;

    EdgeMatrixEntry getBestPath();

    void remove_top(void);
};

struct LowestCostInfo : RowColCost
{
    std::string objectName;
    std::string goalName;
    //int row;
    //int col;
    //double cost;
    //int indexInArray;  // index in the results vector

    LowestCostInfo()
    {
        objectName="";
        goalName="";
    }
};

struct FinalAllocation
{
    ObjectInfo object;
    GoalInfo goal;

    double cost;
    int row;
    int col;
    std::vector<VertexData> vertexChain;

    // The actual states used
    ReloPush::State startPose;
    ReloPush::State goalPose;

    // planning context snapshot
    PlanningContext snapshot;

    std::vector<EdgeData> paths; // contains edge information inc. mode
    ReloPush::StatePathPtrList transitPaths;

    EdgePathList obsReloPaths;

    std::pair<ReloPush::StatePathPtr,std::vector<size_t>> toSinglePathPtr(double interpolation_resolution = 0.1);

    double getPushingLength(void) const;
};

class FinalTaskSequence
{
public:
    std::vector<FinalAllocation> task_sequence;

    std::vector<ReloPush::State> to_StateList(void);
};


/**
 * @brief Scans an Eigen::MatrixXd for its minimal value (if any).
 *        Returns (value, row, col). If matrix is empty, returns +inf.
 */
MatrixMinEntry findMatrixMin(const Eigen::MatrixXd &mat);

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
    const std::vector<Vertex> &goalVerts);
*/

MatrixResultPtr computeCostMatrixWithPaths(
    const Graph &g,
    const std::vector<Vertex> &objectVerts,
    const std::vector<Vertex> &goalVerts,
    PlanningContext& ctx);

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
    std::unordered_map<std::string, ObjectGoalPair> &pairs);
*/
std::map<std::string, PairCostResult> computeMatrixPairs(
    const Graph &g, std::unordered_map<std::string, ObjectGoalPair> &pairs, PlanningContext& ctx);

/**
 * @brief Finds the single lowest cost among all PairCostResult entries,
 *        returning its details (object, goal, row, col, cost, and index).
 *
 * @param results The vector of PairCostResult from computeAndSortAllPairs().
 * @return A std::pair<LowestCostInfo, PairCostResult> with the absolute minimal cost found.
 *         If 'results' is empty, fields will be default/invalid.
 */
LowestCostInfo findAbsoluteLowestCost(std::map<std::string, PairCostResult> &results);

typedef std::map<std::string, PairCostResult> PairResultsMap;

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
                          const ReloPush::State &objNewState);

// ---------------------------------------------------------------------------
// Helper Function 3: One iteration of picking the best pair and planning
// ---------------------------------------------------------------------------
bool findFeasibleAllocation(PairResultsMap &pairResults,
                            const std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                            PlanningContext &planCtx,
                            std::vector<EdgePath> &ObsReloPathList,
                            LowestCostInfo &bestPick,
                            std::unordered_map<std::string, ReloPush::State> &ToUpdate,
                            std::string &failedObjectName, ObjectMap objects);

// ---------------------------------------------------------------------------
// Helper Function 4: The main planning/allocation loop
// ---------------------------------------------------------------------------
bool performAllocations(const WorkspaceBoundary &boundary,
                        std::unordered_map<std::string, ObjectInfo> &objects,
                        std::unordered_map<std::string, GoalInfo> &goals,
                        std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                        std::vector<FinalAllocation> &finalSequence,
                        bool& use_opt);

// ---------------------------------------------------------------------------
// Helper Function 5: Print the final sequence
// ---------------------------------------------------------------------------
void printFinalSequence(const std::vector<FinalAllocation> &finalSequence);

#endif // TASKALLOCATION_HPP
