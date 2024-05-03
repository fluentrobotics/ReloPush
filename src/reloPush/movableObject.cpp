#include <reloPush/movableObject.h>

double normalizeAngle(double angle) {
    // Normalize angle to be within the range of 0 to 2pi
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle >= 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}


VertexStatePair::VertexStatePair(){}

VertexStatePair::VertexStatePair(VertexPtr v_in, StatePtr s_in) : vertex(v_in), state(s_in) {}

VertexStatePair::VertexStatePair(Vertex v_in, State s_in)
{
    vertex = std::make_shared<Vertex>(v_in);
    state = std::make_shared<State>(s_in);
}

VertexStatePair::VertexStatePair(Vertex v_in, StatePtr s_in) : state(s_in)
{
    vertex = std::make_shared<Vertex>(v_in);
}

VertexStatePair::VertexStatePair(VertexPtr v_in, State s_in) : vertex(v_in)
{
    state = std::make_shared<State>(s_in);
}


movableObject::movableObject(){}

movableObject::movableObject(float x_in, float y_in, float th_in, std::string name_in, float n_side_in, GraphPtr graph_ptr) 
                                : x(x_in), y(y_in), th(th_in), name(name_in), n_side(n_side_in)
{
    pushing_poses.resize(n_side);
    vertex_names.resize(n_side);

    float ori_incre = M_PI * 2 / (float)n_side; // orientation increment
    
    for(size_t i = 0; i<n_side; i++)
    {
        //pushing pose: orientation and translational offset
        //todo: get offset as a param
        float trans_offset = 0;//0.425f; //0.125 + 0.3
        float push_ori = normalizeAngle(i*ori_incre + th);

        float x_push = x - (trans_offset*cos(push_ori));
        float y_push = y - (trans_offset*sin(push_ori));

        pushing_poses[i] = std::make_shared<State>(State(x_push, y_push, push_ori));
        vertex_names[i] = name + "_" + std::to_string(i); // {block_name}_{node_index}
    }

    if(graph_ptr)
        add_to_graph(graph_ptr);
}

void movableObject::add_to_graph(GraphPtr graph)
{
    vertex_state_list.resize(n_side);
    for(size_t i=0; i<n_side; i++)
    {
        auto temp_vertex = boost::add_vertex(vertex_names[i], *graph);
        vertex_state_list[i] = VertexStatePair(temp_vertex,pushing_poses[i]);
    }
}


NameMatcher::NameMatcher(){}
NameMatcher::NameMatcher(std::vector<movableObject> mo_list, bool clear_map)
{
    if(clear_map){
        vsMap.clear();
        moMap.clear();
    }

    for(auto& it : mo_list)
    {
        moMap.insert({it.name,std::make_shared<movableObject>(it)});
        for(size_t n=0; n<it.vertex_names.size(); n++)
        {
            vsMap.insert({it.vertex_names[n], it.vertex_state_list[n]});
            moMap.insert({it.vertex_names[n], std::make_shared<movableObject>(it)});
        }
    }
}

movableObjectPtr NameMatcher::getObject(std::string obj_name)
{
    return moMap[obj_name];
}