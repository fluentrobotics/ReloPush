#ifndef MOVABLEOBJECT_H
#define MOVABLEOBJECT_H

#include <string>

class movableObject
{
    public:
        float x;
        float y;
        float n_side;
        float th;
        std::string name;

        movableObject(float x_in, float y_in, float th_in, std::string name_in = "",float n_side_in = 4) 
                                        : x(x_in), y(y_in), th(th_in), name(name_in), n_side(n_side_in)
        {}
};

#endif