#ifndef EDGE_MATCHER_H
#define EDGE_MATCHER_H

#include<graphTools/graph_info.h>
#include<omplTools/dubins_tools.h>

#include <boost/functional/hash.hpp>

#include <omplTools/State.h>

namespace std {
    template <>
    struct hash<Edge> {
        size_t operator()(const Edge& e) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, e); // Combine hash of e with seed
            return seed;
        }
    };
}

namespace graphTools
{
    class EdgePathInfo
    {
        public:
            //from/to vertex
            vertexPair vertices;
            //from/to states
            State sourceState;
            State targetState;
            //dubins set
            dubinsPath path;
            //cost
            float cost;
            //path classification
            pathType path_class;

            EdgePathInfo();
            EdgePathInfo(Vertex source_v, Vertex target_v, State source_s, State target_s, dubinsPath path_in, pathType path_class_in, Edge edge_in, GraphPtr gPtr);
            EdgePathInfo(vertexPair pair_in, State source_s, State target_s, dubinsPath path_in, pathType path_class_in);
    };

    class EdgeMatcher
    {
        private:
            std::unordered_map<Edge,EdgePathInfo> edgeMap;

        public:
            EdgeMatcher();
            void insert(Edge e_in, EdgePathInfo p_in);
            std::shared_ptr<std::vector<std::pair<Edge,EdgePathInfo>>> get_entries();
            EdgePathInfo getPath(Edge e_in);
    };
}

    #endif