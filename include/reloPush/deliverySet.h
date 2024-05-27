#ifndef RELOCATION_SET_H
#define RELOCATION_SET_H

#include <vector>
#include <memory>

#include <omplTools/State.h>
#include <reloPush/movableObject.h>

typedef std::vector<std::pair<std::string, State>> relocationPair_list; //index of the mo_list, new state
typedef std::shared_ptr<statePath> pathPtr;
typedef std::shared_ptr<std::vector<statePath>> pathsPtr;

class deliverySet
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

        deliverySet(){}

        deliverySet(std::vector<movableObject>& mo_list_in, relocationPair_list relocPairs_in, std::string delivery_name, pathPtr pathPtr_in)
        {
            mo_list = std::vector<movableObject>(mo_list_in);
            relocPairs = relocPairs_in;
            delivery_obj_name = delivery_name;
            reloPath = pathPtr_in;
        }

};

#endif