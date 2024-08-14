#ifndef DATA_COLLECTOR_H
#define DATA_COLLECTOR_H

#include <vector>
#include <reloPush/stopwatch.h>
#include <omplTools/State.h>
#include <memory>
#include <string>

enum moveType {pre,temp,final,app}; // push, push, push, non-push

std::string moveTypeToStr(moveType mt);

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

    // Print Path
    void print_path();
    
};

class PathInfoList
{
    public:
    std::vector<PathInfo> paths;

    PathInfoList();
    void push_back(PathInfo& p_in);
    void append(PathInfoList& list_in);
    void print_seq(void);
    size_t count_pre_relocations(void);
    size_t count_temp_relocations(void);
    size_t count_total_relocations(void);
    float total_path_length();
    float print_path();
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
    PathInfoList pathInfoList;

    DataCollector()
    {}

    DataCollector(stopWatchSet& watches_in, PathInfoList& list_in) : stopWatches(watches_in), pathInfoList(list_in)
    {}

    void append_pathinfo(PathInfoList& list_in);

};

typedef std::shared_ptr<DataCollector> collectorPtr;

#endif