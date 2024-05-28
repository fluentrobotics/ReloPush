#ifndef RELOCATION_SET_H
#define RELOCATION_SET_H

#include <vector>
#include <memory>

#include <omplTools/State.h>
#include <reloPush/movableObject.h>

typedef std::vector<std::pair<std::string, State>> relocationPair_list; //index of the mo_list, new state
typedef std::shared_ptr<statePath> pathPtr;
typedef std::shared_ptr<std::vector<statePath>> pathsPtr;

// context of each delivery
class deliveryContext
{
    public:
        // movable objects before relocation
        std::vector<movableObject> mo_list;

        // relocation pair - mo / new state
        relocationPair_list relocPairs;

        // index of delivering object
        //size_t delivery_obj_ind;
        std::string delivery_obj_name;

        // relocating path
        pathPtr reloPath;

        deliveryContext(){}

        deliveryContext(std::vector<movableObject>& mo_list_in, relocationPair_list relocPairs_in, std::string delivery_name, pathPtr pathPtr_in)
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

        statePath serializePath()
        {
            statePath out_path;
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