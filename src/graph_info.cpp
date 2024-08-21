//#include <graphTools/graph_info.h>
#include <graphTools/edge_path_info.h>

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
    vertexPair::vertexPair() {};

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

    Vertex getVertex(const GraphPtr gPtr, const std::string& name)
    {
        Graph g = *gPtr;
        return getVertex(g,name);
    }
    Vertex getVertex(const Graph& graph, const std::string& name)
    {
        boost::graph_traits<Graph>::vertex_iterator vi, vend;
        Vertex vertex;
        auto nameMap = boost::get(boost::vertex_name, graph);
        for (boost::tie(vi, vend) = boost::vertices(graph); vi != vend; vi++)
        {
            if (nameMap[*vi] == name)
                vertex = *vi;
        }
        return vertex;
    }

    EdgePathInfo::EdgePathInfo() {};

    EdgePathInfo::EdgePathInfo(Vertex source_v, Vertex target_v, State source_s, State target_s,
                                reloDubinsPath path_in, preRelocList& pre_relocs, pathType path_class_in,Edge edge_in, GraphPtr gPtr) 
    : path(path_in), path_class(path_class_in), pre_relocations(pre_relocs)
    {
        vertices = vertexPair(source_v,target_v,edge_in,gPtr);
        cost = path_in.lengthCost();
        sourceState = source_s;
        targetState = target_s;
    }

    EdgePathInfo::EdgePathInfo(vertexPair pair_in, State source_s, State target_s, reloDubinsPath path_in, preRelocList& pre_relocs, pathType path_class_in) 
    : vertices(pair_in), path(path_in), path_class(path_class_in), pre_relocations(pre_relocs)
    {
        cost = path_in.lengthCost();
        sourceState = source_s;
        targetState = target_s;
    }

    //update with new dubins path keeping vertices, path_class and pre_relocations
    void EdgePathInfo::update_dubins(reloDubinsPath& reloDubins)
    {
        cost = reloDubins.lengthCost();
        sourceState = reloDubins.startState;
        targetState = reloDubins.targetState;
        path = reloDubins;
    }


    EdgeMatcher::EdgeMatcher() {
        edgeMap.clear();
    }


    void EdgeMatcher::insert(Edge e_in, EdgePathInfo p_in)
    {
        edgeMap.insert({e_in,p_in});
    }

    std::shared_ptr<std::vector<std::pair<Edge,EdgePathInfo>>> EdgeMatcher::get_entries()
    {
        std::vector<std::pair<Edge,EdgePathInfo>> out_vec(0);

        // iterate through unordered map
        for (auto i = edgeMap.begin(); i != edgeMap.end(); i++)
        {
            out_vec.push_back({i->first,i->second});
        }

        return std::make_shared<std::vector<std::pair<Edge,EdgePathInfo>>>(out_vec);
    }

    EdgePathInfo EdgeMatcher::getPath(Edge e_in)
    {
        return edgeMap[e_in];
    }

    void EdgeMatcher::reset()
    {
        edgeMap.clear();
    }

}
