//
// Created by Magnus Ödman on 2017-09-10.
//

#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H


#include "statemachine.h"
#include <math.h>


class trajectorygenerator {

public:
    std::tuple<std::vector<double>, std::vector<double>> GenerateTrajectory(car ego, statemachine sm, std::vector<double> previous_path_x, std::vector<double> previous_path_y, std::vector<double> map_waypoints_s,
                                std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y);
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
