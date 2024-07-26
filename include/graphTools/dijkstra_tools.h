// Base code from https://github.com/siavashk/lignum-vitae

#ifndef DIJKSTRA_TOOLS_H
#define DIJKSTRA_TOOS_H

#include "graph_info.h"

class pathFinder {
    public:
        Vertex getVertexFromString(
            const Graph& graph,
            const std::string& name
        );

        std::vector<Vertex> getPath(
            const Graph& graph,
            const std::vector<Vertex>& pMap,
            const Vertex& source,
            const Vertex& destination
        );

        std::pair<std::vector<Vertex>, float> djikstra(
            const GraphPtr graph,
            const std::string& sourceName,
            const std::string& destinationName
        );

        void printPath(
            const GraphPtr graphPtr,
            const std::vector<Vertex>& path
        );
    //private:
    //    bgl_named_params distanceMap;

};

class graphPlanResult
{
    public:
        std::string sourceVertexName;
        std::string targetVertexName;
        float cost;
        std::vector<Vertex> path;
        std::string object_name;
        size_t min_row; // row with min cost
        size_t min_col;

        graphPlanResult()
        {}
        graphPlanResult(std::string s_in, std::string t_in, float c_in, std::vector<Vertex> p_in, std::string name_in, size_t delivery_obj_ind, size_t min_col_in) 
        : sourceVertexName(s_in), targetVertexName(t_in), cost(c_in), path(p_in), object_name(name_in), min_row(delivery_obj_ind), min_col(min_col_in)
        {}
};

#endif // !DIJKSTRA_TOOLS_H
