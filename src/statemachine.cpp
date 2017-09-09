//
// Created by Magnus Ödman on 2017-09-05.
//

#include "statemachine.h"
#include <vector>
#include <iostream>
#include <math.h>


enum LANE {LEGFT = 0, MIDDLE = 1, RIGHT=2 };

void statemachine::Update(car ego, std::vector<car> cars) {

  switch (state) {
    case CHANGE_LANE_LEFT:
      updateLaneShift(ego);
      break;
    case CHANGE_LANE_RIGHT:
      updateLaneShift(ego);
      break;
    default:
      keepLane(ego, cars);
  }
}


void statemachine::keepLane(car ego, std::vector<car> cars) {
  bool to_close = false;
  double speed_ahead = ref_vel;

  for (int index = 0; index < cars.size(); index++) {
    auto other_car = cars[index];
    double diff_d = sqrt(pow(ego.d - other_car.d,2));
    double diff_s = other_car.s - ego.s;
    if (other_car.d < (2+4*lane+2) && other_car.d > (2 + 4*lane-1)) {


      if(diff_s < 40 and diff_s > 0) {
        //std::cout << other_car.id << ", " << diff_s << ", " << other_car.speed << std::endl;
        if(other_car.speed < ego.speed) {

          to_close = true;
          if(speed_ahead > other_car.speed) {
            speed_ahead = other_car.speed*.95;
            //std::cout << "BLOCKED BY " << other_car.id << " DIST: " << diff_s << "[ " << speed_ahead << "]" << std::endl;
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
  bool right_can_change = true;
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
        if(diff_s < 15 and diff_s > -10 ) {
          right_can_change = false;
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





  std::cout << "Current lane: " << lane << "State: " << state << " LANES: " << "|" << (left_lane_free? " ": "B") << "|" << (current_lane_free? " ": "B") << "|" << (right_lane_free?" ": "B") << "|" << " s: " << ego.s <<std::endl;



  if(current_lane_free) {
    //
  } else {


    if (left_lane_free) {
      changeLaneLeft();

    }
    else if (right_lane_free) {
      changeLaneRight();
    } else {
      if (lane == 0 && right_can_change) {
        changeLaneRight();
      } else if( lane == 2 && left_can_change) {
        changeLaneLeft();
      }
    }
  }

  if (!to_close) {
    //Accelerate
    if(this-> ref_vel < 49.5) {
      ref_vel += 0.224;
    }
  } else {

    if(ref_vel > speed_ahead * 2.24) {
      ref_vel -= 0.224;
    }
  }
}

void statemachine::updateLaneShift(car ego) {
  double centerLine = (lane + 1) * 4 - 2;
  double distance_to_center_line = sqrt(pow(ego.d - centerLine, 2));
  if(distance_to_center_line < 1) {
    state = KEEP_LANE;

  }
  std::cout << "Current lane: " << lane << " State: " << state << " distance to center: " << distance_to_center_line << std::endl;

}

void statemachine::changeLaneRight() {
  state = CHANGE_LANE_RIGHT;
  lane = lane + 1;


}

void statemachine::changeLaneLeft() {

  lane = lane - 1;
  state = CHANGE_LANE_LEFT;

 }
