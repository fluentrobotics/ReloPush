#ifndef TASKALLOCATION_HPP
#define TASKALLOCATION_HPP

#include <Eigen/Dense>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <GraphData.hpp>
#include <GraphBuilder.hpp>

#include <ObjectInfo.hpp>

#include <string>
#include <memory>

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
};

using EdgePathList = std::vector<EdgePathPtr>;
struct MatrixResult
{
    // The full cost matrix (row = object vertex, col = goal vertex).
    Eigen::MatrixXd costMat;

    // A sorted list of (row, col, cost) in ascending order of cost.
    std::vector<RowColCost> sortedEntries;

    // A 2D array (size = [Nobj x Ngoal]) storing EdgePathPtr
    std::vector<std::vector<EdgePathList>> pathMat;
};
using MatrixResultPtr = std::shared_ptr<MatrixResult>;

class PairCostResult
{
public:
    std::string objectName;
    std::string goalName;

    double bestCost;
    int bestRow;
    int bestCol;

    // The entire MatrixResult, which has costMat + sortedEntries
    MatrixResultPtr matrixResult;

    EdgePathList getBestPath();

    void remove_top(void);
};

struct LowestCostInfo
{
    std::string objectName;
    std::string goalName;
    int row;
    int col;
    double cost;
    //int indexInArray;  // index in the results vector
};

struct FinalAllocation
{
    ObjectInfo object;
    GoalInfo goal;

    double cost;
    int row;
    int col;

    // The actual states used
    ReloPush::State startPose;
    ReloPush::State goalPose;

    // Pre-relocation info
    bool usedPreRelocation = false;
    double xRelocated      = 0.0;
    double yRelocated      = 0.0;
    double preReloCost     = 0.0;
    int relocatingIndex    = -1; // or double relocatingAngle

    std::vector<EdgePathPtr> paths;

    ReloPush::StatePathPtr toSinglePathPtr(double interpolation_resolution = 0.1);
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

MatrixResult computeCostMatrixWithPaths(
    const Graph &g,
    const std::vector<Vertex> &objectVerts,
    const std::vector<Vertex> &goalVerts);

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
    const Graph &g, std::unordered_map<std::string, ObjectGoalPair> &pairs);

/**
 * @brief Finds the single lowest cost among all PairCostResult entries,
 *        returning its details (object, goal, row, col, cost, and index).
 *
 * @param results The vector of PairCostResult from computeAndSortAllPairs().
 * @return A LowestCostInfo with the absolute minimal cost found.
 *         If 'results' is empty, fields will be default/invalid.
 */
LowestCostInfo findAbsoluteLowestCost(std::map<std::string, PairCostResult> &results);


#endif // TASKALLOCATION_HPP
