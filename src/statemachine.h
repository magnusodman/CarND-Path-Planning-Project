//
// Created by Magnus Ödman on 2017-09-05.
//

#ifndef PATH_PLANNING_STATEMACHINE_H
#define PATH_PLANNING_STATEMACHINE_H


#include <vector>

struct car {
    long id;
    double x;
    double y;
    double speed;
    double s;
    double d;

};

class statemachine {

public:
    int lane = 1;
    double ref_vel = 0;

    void Update(car ego, std::vector<car> cars);
};



#endif //PATH_PLANNING_STATEMACHINE_H
