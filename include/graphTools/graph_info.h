#ifndef GRAPH_INFO_H
#define GRAPH_INFO_H

#include <iostream>
#include <memory>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <omplTools/dubins_tools.h>

using namespace boost;

typedef std::string vertexNameType;
typedef float weightType;

typedef adjacency_list<listS, vecS, directedS,
    property<vertex_name_t, vertexNameType>,
    property<edge_weight_t, weightType>> Graph;

typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef std::shared_ptr<Vertex> VertexPtr;
typedef graph_traits<Graph>::edge_descriptor Edge;
typedef std::vector<Edge> edgeVec;
typedef std::shared_ptr<edgeVec> edgeVecPtr;
typedef std::shared_ptr<Graph> GraphPtr;

namespace graphTools
{
    class vertexPair
    {
    public:
        vertexPair();
        vertexPair(Vertex source_in, Vertex sink_in, Edge edge_in, GraphPtr graph_ptr_in);
        Vertex getSource();
        Vertex getSink();
        vertexNameType getSourceName();
        vertexNameType getSinkName();
        Edge getEdge();
        weightType getEdgeWeight();
        GraphPtr getGraphPtr();

    private:
        Vertex source_vertex;
        Vertex sink_vertex;
        Edge edge;
        weightType edgeWeight;
        GraphPtr graph_ptr;
    };

    // get all edges from the input graph
    edgeVecPtr getEdges(GraphPtr graph);
    edgeVecPtr getEdges(Graph& graph);

    // find the source and sink vertices
    vertexPair getVertexPair(Edge& edge, GraphPtr graph);
    vertexPair getVertexPair(Edge& edge, Graph& graph);
    vertexPair getVertexPair(edgeVecPtr edges, size_t ind, GraphPtr graph);
    vertexPair getVertexPair(edgeVecPtr edges, size_t ind, Graph& graph);

    // get name of a vertex
    vertexNameType getVertexName(Vertex v, GraphPtr graph);
    vertexNameType getVertexName(Vertex v, Graph& graph);

    class pathInfo
    {
        public:
            //from/to vertex
            vertexPair vertices;
            //dubins set
            dubinsPath path;
            //cost
            float cost;
            //path classification
            std::string class_str;

            pathInfo();
            pathInfo(vertexPair pair_in, dubinsPath path_in, std::string class_str_in);
    };

    class EdgeMatcher
    {
        public:
            std::unordered_map<Edge,pathInfo> edgeMap;

            EdgeMatcher();
            void insert(Edge e_in, pathInfo p_in);
    };
}

#endif // !GRAPH_INFO_H