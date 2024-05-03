// Base code from https://github.com/siavashk/lignum-vitae

#include <dijkstra_tools.h>
#include <reloPush/stopwatch.h>

Vertex pathFinder::getVertexFromString(
    const Graph& graph,
    const std::string& name
) {
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

std::vector<Vertex> pathFinder::getPath(
    const Graph& graph,
    const std::vector<Vertex>& pMap,
    const Vertex& source,
    const Vertex& destination
) {
    std::vector<Vertex> path;
    Vertex current = destination;
    while (current != source)
    {
        path.push_back(current);
        current = pMap[current];
    }
    path.push_back(source);
    return path;
}

std::pair<std::vector<Vertex>, float> pathFinder::djikstra(
    const Graph& graph,
    const std::string& sourceName,
    const std::string& destinationName
) {
    Vertex source = getVertexFromString(graph, sourceName);
    Vertex destination = getVertexFromString(graph, destinationName);

    const int numVertices = boost::num_vertices(graph);
    std::vector<float> distances(numVertices);
    std::vector<Vertex> pMap(numVertices);

    auto distanceMap = boost::predecessor_map(
        boost::make_iterator_property_map(pMap.begin(), boost::get(boost::vertex_index, graph))).distance_map(
            boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, graph)));
    
    //runtime measurement
    stopWatch st(std::string("map"));
    boost::dijkstra_shortest_paths(graph, source, distanceMap); //todo: avoid unnecessary distance map making 
    st.stop_and_get_us();

    // return path and cost
    return std::make_pair(getPath(graph, pMap, source, destination), distances[destination]);
}

void pathFinder::printPath(
    const Graph& graph,
    const std::vector<Vertex>& path
) {
    auto nameMap = boost::get(boost::vertex_name, graph);
    std::cout << "The shortest path between " << nameMap[path.back()] << " and "
        << nameMap[path.front()] << " is: " << std::endl;
    for (auto it = path.rbegin(); it < path.rend(); ++it)
    {
        if (it == path.rend() - 1)
            std::cout << nameMap[*it] << std::endl;
        else
            std::cout << nameMap[*it] << " --> ";
    }
}