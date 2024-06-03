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

    update_pushing_poses();

    if(graph_ptr!=nullptr)
        add_to_graph(graph_ptr);
}

void movableObject::update_pushing_poses()
{
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
}

std::vector<Eigen::Vector2f> movableObject::get_push_unitvecs()
{
    std::vector<Eigen::Vector2f> out_vec(n_side);

    for(size_t n=0; n<n_side; n++)
    {
        auto th_ = jeeho::convertEulerRange_to_pi(pushing_poses[n]->yaw);
        out_vec[n](0) = cosf(th_);
        out_vec[n](1) = sinf(th_);
    }

    return out_vec;
}

float movableObject::get_x()
{
    return x;
}
float movableObject::get_y()
{
    return y;
}
size_t movableObject::get_n_side()
{
    return n_side;
}
float movableObject::get_th()
{
    return th;
}
std::string movableObject::get_name()
{
    return name;
}

void movableObject::set_x(float x_in)
{
    x = x_in;
}
void movableObject::set_y(float y_in)
{
    y = y_in;
}
void movableObject::set_th(float th_in)
{
    th = th_in;
}


std::vector<StatePtr> movableObject::get_pushing_poses()
{
    return pushing_poses;
}
std::vector<std::string> movableObject::get_vertex_names()
{
    return vertex_names;
}
std::vector<VertexStatePair> movableObject::get_vertex_state_list()
{
    return vertex_state_list;
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

    NameMatcher::addVertices(mo_list);
}

movableObjectPtr NameMatcher::getObject(std::string obj_name)
{
    return moMap[obj_name];
}

movableObjectPtr NameMatcher::getObject(Vertex vertex_in)
{
    return vertexMap[vertex_in];
}

vertexStatePairPtr NameMatcher::getVertexStatePair(std::string vertex_name)
{
    return std::make_shared<VertexStatePair>(vsMap[vertex_name]);
}

void NameMatcher::addVertices(std::vector<movableObject>& mo_list)
{
    for(auto& it : mo_list)
    {
        moMap.insert({it.get_name(),std::make_shared<movableObject>(it)});
        auto moPtr = std::make_shared<movableObject>(it);
        for(size_t n=0; n<it.get_vertex_names().size(); n++)
        {
            vsMap.insert({it.get_vertex_names()[n], it.get_vertex_state_list()[n]});
            moMap.insert({it.get_vertex_names()[n], moPtr});
        }

        auto vs_list = it.get_vertex_state_list();
        for(size_t n=0; n<vs_list.size(); n++)
            vertexMap.insert({*vs_list[n].vertex, moPtr});
        
    }
}

void NameMatcher::reset(std::vector<movableObject>& mo_list)
{
    vsMap.clear();
    moMap.clear();
    

    NameMatcher::addVertices(mo_list);
}