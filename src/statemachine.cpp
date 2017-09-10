//
// Created by Magnus Ödman on 2017-09-05.
//

#include "statemachine.h"
#include <iostream>
#include <math.h>


enum LANE {
    LEGFT = 0, MIDDLE = 1, RIGHT = 2
};

void statemachine::Update(car ego, std::vector<car> cars, STATE path) {

  switch (state) {
    case CHANGE_LANE_LEFT:
      updateLaneShift(ego, cars);
      break;
    case CHANGE_LANE_RIGHT:
      updateLaneShift(ego, cars);
      break;
    default:
      keepLane2(ego, cars, path);
  }
}

void statemachine::keepLane2(car ego, std::vector<car> cars, STATE path) {

  switch (path) {
    case CHANGE_LANE_LEFT:
      changeLaneLeft();
      break;
    case CHANGE_LANE_RIGHT:
      changeLaneRight();
      break;
    default:
      break;
  }

  adjustSpeed(ego, cars);

}

void statemachine::adjustSpeed(const car &ego, const std::vector<car> &cars) {
  bool to_close = false;
  double speed_ahead = ref_vel;

  //double distance_to_car_ahead = 40;
  double distance_to_car_ahead = 1.9 * ego.speed * 0.44;

  for (int index = 0; index < cars.size(); index++) {
    auto other_car = cars[index];
    double diff_d = sqrt(pow(ego.d - other_car.d, 2));
    double diff_s = other_car.s - ego.s;
    if (other_car.d<(2 + 4 * lane + 2) && other_car.d>(2 + 4 * lane - 1)) {


      if (diff_s < distance_to_car_ahead and diff_s > 0) {
        //std::cout << other_car.id << ", " << diff_s << ", " << other_car.speed << std::endl;
        if (other_car.speed < ego.speed) {

          to_close = true;
          if (speed_ahead > other_car.speed) {
            speed_ahead = other_car.speed * .95;
            //std::cout << "BLOCKED BY " << other_car.id << " DIST: " << diff_s << "[ " << speed_ahead << "]" << std::endl;
          }
        }

      }
    }
  }

  if (!to_close) {
    //Accelerate
    if (ref_vel < 49.5) {
      ref_vel += 0.224;
    }
  } else {

    if (ref_vel > speed_ahead * 2.24) {
      ref_vel -= 0.224;
    }
  }
}



void statemachine::updateLaneShift(car ego, std::vector<car> cars) {
  double centerLine = (lane + 1) * 4 - 2;
  double distance_to_center_line = sqrt(pow(ego.d - centerLine, 2));
  if (distance_to_center_line < 1) {
    state = KEEP_LANE;

  }
  std::cout << "Current lane: " << lane << " State: " << state << " distance to center: " << distance_to_center_line
            << std::endl;
  adjustSpeed(ego, cars);

}

void statemachine::changeLaneRight() {
  state = CHANGE_LANE_RIGHT;
  lane = lane + 1;


}

void statemachine::changeLaneLeft() {

  lane = lane - 1;
  state = CHANGE_LANE_LEFT;

}
