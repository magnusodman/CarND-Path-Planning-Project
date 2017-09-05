//
// Created by Magnus Ödman on 2017-09-05.
//

#ifndef PATH_PLANNING_STATEMACHINE_H
#define PATH_PLANNING_STATEMACHINE_H


#include <vector>

enum STATE {KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT};

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
    STATE state = KEEP_LANE;
    int lane = 1;
    double ref_vel = 0;

    void Update(car ego, std::vector<car> cars);

    void changeLaneLeft();

    void keepLane(car ego, std::vector<car> cars);

    void updateLaneShift(car ego);

    void changeLaneRight();
};



#endif //PATH_PLANNING_STATEMACHINE_H
