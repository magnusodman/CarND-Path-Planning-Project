//
// Created by Magnus Ödman on 2017-09-05.
//

#ifndef PATH_PLANNING_STATEMACHINE_H
#define PATH_PLANNING_STATEMACHINE_H


#include <vector>

enum STATE {
    KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT
};

struct car {
    long id;
    double x;
    double y;
    double speed;
    double s;
    double d;
    double yaw;
};

class statemachine {

public:
    STATE state = KEEP_LANE;
    int lane = 1;
    double ref_vel = 0;

    void Update(car ego, std::vector<car> cars, STATE state1);

    void changeLaneLeft();

    void keepLane(car ego, std::vector<car> cars, STATE state);

    void updateLaneShift(car ego, std::vector<car> vector);

    void changeLaneRight();

    void keepLane2(car ego, std::vector<car> cars, STATE path);

    void adjustSpeed(const car &ego, const std::vector<car> &cars);
};


#endif //PATH_PLANNING_STATEMACHINE_H
