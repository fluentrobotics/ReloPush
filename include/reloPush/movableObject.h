#ifndef MOVABLEOBJECT_H
#define MOVABLEOBJECT_H

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <unordered_map>
#include <omplTools/State.h>
#include <graphTools/graph_info.h>

/*
double normalizeAngle(double angle) {
    // Normalize angle to be within the range of -pi to pi
    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}
*/

double normalizeAngle(double angle);

class VertexStatePair
{
    public:
        VertexPtr vertex;
        StatePtr state;
        VertexStatePair();

        VertexStatePair(VertexPtr v_in, StatePtr s_in);

        VertexStatePair(Vertex v_in, State s_in);

        VertexStatePair(Vertex v_in, StatePtr s_in);

        VertexStatePair(VertexPtr v_in, State s_in);
};

class movableObject
{
    public:
        float x;
        float y;
        float n_side;
        float th;
        std::string name;
        std::vector<StatePtr> pushing_poses;
        std::vector<std::string> vertex_names;
        std::vector<VertexStatePair> vertex_state_list; // push-pose/vertex pair

        movableObject();

        movableObject(float x_in, float y_in, float th_in = 0, std::string name_in = "", 
                                    float n_side_in = 4, GraphPtr graph_ptr = nullptr);

        void add_to_graph(GraphPtr graph);
};

typedef std::shared_ptr<movableObject> movableObjectPtr;
class NameMatcher
{
    public:
        std::unordered_map<std::string,VertexStatePair> vsMap;
        std::unordered_map<std::string,movableObjectPtr> moMap;

        NameMatcher();
        NameMatcher(std::vector<movableObject> mo_list, bool clear_map = true);
};

#endif