//
// Created by Magnus Ödman on 2017-09-09.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include "statemachine.h"

class pathplanner {

public:
    void PlanPath(car ego, std::vector<car> cars);
};


#endif //PATH_PLANNING_PATHPLANNER_H
