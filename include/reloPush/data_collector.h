#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <vector>
#include <reloPush/stopwatch.h>
#include <omplTools/State.h>
#include <memory>
#include <string>

enum moveType {pre,temp,final,none}; // push, push, push, non-push

class PathInfo
{
    public:
    std::string vertexName;
    moveType type;
    State fromPose;
    State toPose;
    std::vector<State> path;

    PathInfo()
    {
        vertexName = "";
        fromPose = State();
        toPose = State();
        path.clear();
    }

    PathInfo(std::string vertex_name, moveType path_type, State& from_pose, State& to_pose, std::vector<State>& path_in) 
    : vertexName(vertex_name), type(path_type), fromPose(from_pose), toPose(to_pose), path(path_in)
    {}

    bool is_pushing()
    {
        if(type == moveType::none)
            return false;
        else
            return true;
    }

    // Sum of euclidean distances between consecutive waypoints
    float pathLength() 
    {
        float totalLength = 0.0;

        for (size_t i = 1; i < path.size(); ++i) {
            float dx = path[i].x - path[i-1].x;
            float dy = path[i].y - path[i-1].y;
            totalLength += std::sqrt(dx * dx + dy * dy);
        }

        return totalLength;
    }
    
};

class PathInfoList
{
    public:
    std::vector<PathInfo> paths;

    PathInfoList()
    {paths.clear();}

    
};

class DataCollector
{
    public:
    // timewatch list
    // sequence, including temp remove
    // paths sequence with types (path = state path)
    // number of relocation should be able to be extracted from path sequence

    stopWatchSet stopWatches;
    //int num_of_pre_reloc; // relocation of object of relocation
    //int num_of_temp_reloc; // relocation of other blocking objects
    PathInfo pathInfo;

    DataCollector()
    {}

};

#endif