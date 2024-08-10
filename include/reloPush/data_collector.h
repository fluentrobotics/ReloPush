#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <vector>
#include <reloPush/stopwatch.h>
#include <omplTools/State.h>
#include <memory>
#include <string>

enum moveType {pre,temp,final,app}; // push, push, push, non-push

class PathInfo
{
    public:
    std::string vertexName;
    moveType type;
    State fromPose;
    State toPose;
    std::vector<State> path;

    PathInfo();

    PathInfo(std::string vertex_name, moveType path_type, 
            State& from_pose, State& to_pose, std::vector<State>& path_in);
    
    bool is_pushing();

    // Sum of euclidean distances between consecutive waypoints
    float pathLength();
    
};

class PathInfoList
{
    public:
    std::vector<PathInfo> paths;

    PathInfoList();
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

typedef std::shared_ptr<DataCollector> collectorPtr;

#endif