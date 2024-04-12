#include <graphTools/graph_info.h>

edgeVecPtr graphTools::getEdges(GraphPtr graph)
{
    edgeVec edges;
    Graph::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(*graph); ei != ei_end; ++ei) {
        edges.push_back(*ei);
    }

    return std::make_shared<edgeVec>(edges);
}

edgeVecPtr graphTools::getEdges(Graph& graph)
{
    edgeVec edges;
    Graph::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei) {
        edges.push_back(*ei);
    }

    return std::make_shared<edgeVec>(edges);
}

namespace graphTools
{
    vertexPair::vertexPair(Vertex source_in, Vertex sink_in, Edge edge_in, GraphPtr graph_ptr_in)
        : source_vertex(source_in), sink_vertex(sink_in), edge(edge_in), graph_ptr(graph_ptr_in)
    {
        edgeWeight = getEdgeWeight();
    }

    Vertex vertexPair::getSource()
    {
        return source_vertex;
    }

    Vertex vertexPair::getSink()
    {
        return sink_vertex;
    }

    vertexNameType vertexPair::getSourceName()
    {
        return getVertexName(source_vertex, graph_ptr);
    }
    vertexNameType vertexPair::getSinkName()
    {
        return getVertexName(sink_vertex, graph_ptr);
    }

    Edge vertexPair::getEdge()
    {
        return edge;
    }

    weightType vertexPair::getEdgeWeight()
    {
        if (graph_ptr != nullptr)
            return get(boost::edge_weight, *graph_ptr, edge);
        else
            return 0;
    }

    GraphPtr vertexPair::getGraphPtr()
    {
        return graph_ptr;
    }

    vertexPair getVertexPair(Edge& edge, GraphPtr graph)
    {
        return vertexPair(boost::source(edge, *graph), boost::target(edge, *graph), edge, graph);
    }
    vertexPair getVertexPair(Edge& edge, Graph& graph)
    {
        return vertexPair(boost::source(edge, graph), boost::target(edge, graph), edge, std::make_shared<Graph>(graph));
    }

    vertexPair getVertexPair(edgeVecPtr edges, size_t ind, GraphPtr graph)
    {
        return vertexPair(boost::source(edges->at(ind), *graph), boost::target(edges->at(ind), *graph), edges->at(ind), graph);
    }

    vertexPair getVertexPair(edgeVecPtr edges, size_t ind, Graph& graph)
    {
        return vertexPair(boost::source(edges->at(ind), graph), boost::target(edges->at(ind), graph), edges->at(ind), std::make_shared<Graph>(graph));
    }

    vertexNameType getVertexName(Vertex v, GraphPtr graph)
    {
        return get(boost::vertex_name, *graph, v);
    }

    vertexNameType getVertexName(Vertex v, Graph& graph)
    {
        return get(boost::vertex_name, graph, v);
    }
}
