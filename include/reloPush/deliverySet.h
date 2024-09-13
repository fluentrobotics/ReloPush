#ifndef RELOCATION_SET_H
#define RELOCATION_SET_H

#include <vector>
#include <memory>

#include <omplTools/State.h>
#include <reloPush/movableObject.h>

struct RelocationPair
{
    std::string name;
    State from_state;
    State to_state;

    RelocationPair()
    {}

    RelocationPair(std::string name_in, State& from, State&to) : name(name_in), from_state(from), to_state(to)
    {}
};

typedef std::vector<RelocationPair> relocationPairList; //index of the mo_list, new state
typedef std::shared_ptr<StatePath> StatePathPtr;
typedef std::shared_ptr<std::vector<StatePath>> StatePathsPtr;

class ReloPathInfo
{
    public:
    StatePathPtr statePathPtr;
    std::string vertexName;
    State fromPose;
    State toPose;

    ReloPathInfo()
    {
        statePathPtr = nullptr;
        vertexName = "";
        fromPose = State();
        toPose = State();
    }

    ReloPathInfo(StatePathPtr pathPtr, std::string vName_in, State from_pose, State to_pose) 
                : statePathPtr(pathPtr), vertexName(vName_in), fromPose(from_pose), toPose(to_pose)
    {}
};

class ReloPathInfoList
{
    public:
    std::vector<ReloPathInfo> reloPathInfoList;
};

// context of each delivery
class deliveryContext
{
    public:
        // movable objects before relocation
        std::vector<movableObject> mo_list;

        // relocation pair - mo / new state
        relocationPairList relocPairs;

        // index of delivering object
        //size_t delivery_obj_ind;
        std::string delivery_obj_name;

        // relocating path
        StatePathPtr reloPath;

        deliveryContext(){}

        deliveryContext(std::vector<movableObject>& mo_list_in, relocationPairList relocPairs_in, std::string delivery_name, StatePathPtr pathPtr_in)
        {
            mo_list = std::vector<movableObject>(mo_list_in);
            relocPairs = relocPairs_in;
            delivery_obj_name = delivery_name;
            reloPath = pathPtr_in;
        }

};

class deliveryContextSet
{
    public:
        std::vector<std::shared_ptr<deliveryContext>> delivery_contexts;

        StatePath serializePath()
        {
            StatePath out_path;
            for(auto& it : delivery_contexts)
            {
                for(auto& it2 : *(it->reloPath))
                {
                    out_path.push_back(it2);
                }
            }

            return out_path;
        }
};

#endif