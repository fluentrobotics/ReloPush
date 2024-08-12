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
    for(auto& it : paths)
    {
        std::cout << "<- " << it.vertexName << "(" << moveTypeToStr(it.type) << ") ";
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

void DataCollector::append_pathinfo(PathInfoList& list_in)
{
    pathInfoList.paths.insert(pathInfoList.paths.end(),
                             list_in.paths.begin(),
                             list_in.paths.end());
}