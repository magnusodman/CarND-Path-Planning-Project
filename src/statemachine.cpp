//
// Created by Magnus Ödman on 2017-09-05.
//

#include "statemachine.h"
#include <vector>
#include <iostream>
#include <math.h>


void statemachine::Update(car ego, std::vector<car> cars) {
  bool to_close = false;
  double speed_ahead = ref_vel;

  for (int index = 0; index < cars.size(); index++) {
    auto other_car = cars[index];
    double diff_d = sqrt(pow(ego.d - other_car.d,2));
    double diff_s = other_car.s - ego.s;
    if (other_car.d < (2+4*lane+2) && other_car.d > (2 + 4*lane-1)) {


      if(diff_s < 40 and diff_s > 0) {
        std::cout << other_car.id << ", " << diff_s << ", " << other_car.speed << std::endl;
        if(other_car.speed < ego.speed) {

          to_close = true;
          if(speed_ahead > other_car.speed) {
            speed_ahead = other_car.speed;
            std::cout << "BLOCKED BY " << other_car.id << " DIST: " << diff_s << "[ " << speed_ahead << "]" << std::endl;
          }
        }

      }
    }
  }

  bool left_lane_free = true;
  bool left_can_change = true;
  if(lane == 0) {
    left_lane_free = false;
  }
  else {
    for (int index = 0; index < cars.size(); index++) {
      auto other_car = cars[index];
      double diff_d = sqrt(pow(ego.d - other_car.d, 2));
      double diff_s = other_car.s - ego.s;
      if (other_car.d < (lane * 4) && other_car.d > (lane-1) * 4) {
        if (diff_s < 60 and diff_s > -20) {
          left_lane_free = false;
        }
        if(diff_s < 15 and diff_s > -10 ) {
          left_can_change = false;
        }
      }
    }
  }

  bool right_lane_free = true;
  if(lane == 2) {
    right_lane_free = false;
  } else {
    for (int index = 0; index < cars.size(); index++) {
      auto other_car = cars[index];
      double diff_d = sqrt(pow(ego.d - other_car.d, 2));
      double diff_s = other_car.s - ego.s;
      if (other_car.d < ((lane+2) * 4) && other_car.d > (lane+1) * 4) {
        if (diff_s < 60 and diff_s > -20) {
          right_lane_free = false;
        }
      }
    }

  }


  bool current_lane_free = true;

  for (int index = 0; index < cars.size(); index++) {
    auto other_car = cars[index];
    double diff_d = sqrt(pow(ego.d - other_car.d, 2));
    double diff_s = other_car.s - ego.s;
    if (other_car.d<((lane + 1) * 4) && other_car.d>(lane) * 4) {
      if (diff_s < 60 and diff_s > -20) {
        current_lane_free = false;
      }
    }
  }




  std::cout << "LANES: " << "|" << (left_lane_free? " ": "B") << "|" << (current_lane_free? " ": "B") << "|" << (right_lane_free?" ": "B") << "|" <<std::endl;


  if (!to_close) {
    if(lane == 0 && right_lane_free) {
      lane = 1;
    }
    if(this-> ref_vel < 49.5) {
      ref_vel += 0.224;
    }
  } else {
    if(lane == 0 && right_lane_free) {
      lane = 1;
    }
    if(left_lane_free) {
      lane = lane - 1;

    }
    if(ref_vel > speed_ahead * 2.24) {
      ref_vel -= 0.224;
    }
  }

}
