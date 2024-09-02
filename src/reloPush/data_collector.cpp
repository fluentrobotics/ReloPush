#include <reloPush/data_collector.h>

std::string moveTypeToStr(moveType mt) {
    switch(mt) {
        case pre:   return "pre";
        case temp:  return "temp";
        case final: return "final";
        case app:   return "app";
        default:    return "Unknown";
    }
}

PathInfo::PathInfo()
{
    vertexName = "";
    fromPose = State();
    toPose = State();
    path.clear();
}

PathInfo::PathInfo(std::string vertex_name, moveType path_type, State& from_pose, State& to_pose, std::vector<State>& path_in) 
: vertexName(vertex_name), type(path_type), fromPose(from_pose), toPose(to_pose), path(path_in)
{}

PathInfo::PathInfo(std::string vertex_name, moveType path_type, PathPlanResultPtr planned_path)
: vertexName(vertex_name), type(path_type), fromPose(planned_path->start_pose), toPose(planned_path->goal_pose)
{
    path = planned_path->getPath(true);
}

bool PathInfo::is_pushing()
{
    if(type == moveType::app)
        return false;
    else
        return true;
}

// Sum of euclidean distances between consecutive waypoints
float PathInfo::pathLength() 
{
    float totalLength = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        float dx = path[i].x - path[i-1].x;
        float dy = path[i].y - path[i-1].y;
        totalLength += std::sqrt(dx * dx + dy * dy);
    }

    return totalLength;
}

void PathInfo::print_path()
{
    for(auto& it : path)
    {
        std::cout << it.x << "," << it.y << "," << it.yaw << std::endl;
    }
}

PathInfoList::PathInfoList()
{
    paths.clear();
}

void PathInfoList::push_back(PathInfo& p_in)
{
    paths.push_back(p_in);
}

void PathInfoList::append(PathInfoList& list_in)
{
    paths.insert(paths.end(), list_in.paths.begin(), list_in.paths.end());
}

void PathInfoList::print_seq()
{
    std::cout << "robot ";
    for(auto& it : paths)
    {
        std::cout << "-> " << it.vertexName << "(" << moveTypeToStr(it.type) << ") ";
    }
    std::cout << std::endl;
}

size_t PathInfoList::count_pre_relocations(void)
{
    size_t out_count = 0;
    for(auto& it : paths)
    {
        if(it.type == moveType::pre)
            out_count++;
    }
    return out_count;
}
size_t PathInfoList::count_temp_relocations(void)
{
    size_t out_count = 0;
    for(auto& it : paths)
    {
        if(it.type == moveType::temp)
            out_count++;
    }
    return out_count;
}
size_t PathInfoList::count_total_relocations(void)
{
    return (count_pre_relocations() + count_temp_relocations());
}

float PathInfoList::total_path_length()
{
    size_t out_length = 0;
    for(auto& it: paths)
    {
        out_length += it.pathLength();
    }
    return out_length;
}

float PathInfoList::print_path()
{
    for(auto& it : paths)
    {
        std::cout << it.vertexName << "/" << moveTypeToStr(it.type) << std::endl;
        it.print_path();
    }
}

StatePathPtr PathInfoList::serializedPath()
{
    StatePath out_path(0);
    for(auto& it : paths)
    {
        out_path.insert(out_path.end(), it.path.begin(), it.path.end());
    }

    return std::make_shared<StatePath>(out_path);
}

std::pair<StatePathPtr,StrVecPtr> PathInfoList::serializedPathWithMode()
{
    StatePath out_path(0);
    StrVec out_strvec(0);
    for(auto& it : paths)
    {
        out_path.insert(out_path.end(), it.path.begin(), it.path.end());

        std::string mode_str = "p"; // default: pusing
        if(it.type == moveType::app) // non-pushing
            mode_str = "n";
        
        StrVec temp_vec(it.path.size(),mode_str);
        out_strvec.insert(out_strvec.end(), temp_vec.begin(), temp_vec.end());
    }

    return std::make_pair(std::make_shared<StatePath>(out_path),std::make_shared<StrVec>(out_strvec));
}

std::string PathInfoList::serializeAllinStr()
{

    auto final_nav_path_pair = serializedPathWithMode();

    // send to mocap_tf. mocap_tf will send to controller
    

    std::string out_str = "";

    for(size_t n=0; n < final_nav_path_pair.first->size(); n++)
    {
        State& it = final_nav_path_pair.first->at(n);
        auto x_hex_str = jeeho::float2hexstr(it.x);
        auto y_hex_str = jeeho::float2hexstr(it.y);
        auto yaw_hex_str = jeeho::float2hexstr(it.yaw);

        std::string temp_str = x_hex_str + jeeho::elem_delim + y_hex_str + jeeho::elem_delim + yaw_hex_str; // x,y,yaw
        out_str += temp_str;
        if(n!=final_nav_path_pair.first->size()-1)
            out_str += jeeho::data_delim;
    }

    out_str += jeeho::group_delim;

    for(size_t n=0; n < final_nav_path_pair.second->size(); n++)
    {
        std::string& it = final_nav_path_pair.second->at(n);

        out_str += it;
        if(n!=final_nav_path_pair.second->size()-1)
            out_str += jeeho::data_delim;
    }

    // add timing. First pose is 0
    out_str += jeeho::group_delim + jeeho::float2hexstr(0) + jeeho::data_delim;
    float last_t = 0;
    // do not use MP
    for (size_t n = 1; n < final_nav_path_pair.first->size(); n++)
    {
        // delta distance
        float dx = final_nav_path_pair.first->at(n).x - final_nav_path_pair.first->at(n - 1).x;
        float dy = final_nav_path_pair.first->at(n).y - final_nav_path_pair.first->at(n - 1).y;
        // double dz = out_path.poses[n].pose.position.z - out_path.poses[n-1].pose.position.z;
        float dist = std::sqrt(dx * dx + dy * dy);

        // delta time
        float dTime = dist / Constants::speed_limit;
        last_t += dTime;
        out_str += jeeho::float2hexstr(last_t);
        if (n != final_nav_path_pair.first->size() - 1)
            out_str += jeeho::data_delim;
    }

    // add frame name
    out_str += jeeho::group_delim + params::world_frame;

    return out_str;
}

void DataCollector::append_pathinfo(PathInfoList& list_in)
{
    pathInfoList.paths.insert(pathInfoList.paths.end(),
                             list_in.paths.begin(),
                             list_in.paths.end());
}