#ifndef GRAPH_INFO_H
#define GRAPH_INFO_H

#include <iostream>
#include <memory>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

//#include <omplTools/dubins_tools.h>

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
        GraphPtr graph_ptr = nullptr;
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

    //get vertex from sting name
    Vertex getVertex(const GraphPtr gPtr, const std::string& name);
    Vertex getVertex(const Graph& graph, const std::string& name);
}

#endif // !GRAPH_INFO_H