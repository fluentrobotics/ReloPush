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
            const Graph& graph,
            const std::string& sourceName,
            const std::string& destinationName
        );

        void printPath(
            const Graph& graph,
            const std::vector<Vertex>& path
        );
    //private:
    //    bgl_named_params distanceMap;

};

#endif // !DIJKSTRA_TOOLS_H
