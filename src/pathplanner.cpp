//
// Created by Magnus Ödman on 2017-09-09.
//

#include <math.h>
#include <iostream>
#include "pathplanner.h"

void pathplanner::PlanPath(car ego, std::vector<car> cars) {

  double dt = 1.0;

  double speed_m_s = ego.speed * .447;

  std::vector< std::vector<char>> maze;
  //Check 5 seconds into the future
  for(int seconds = 0; seconds < 8; seconds++) {
    double s_min = -10.0 + ego.s + speed_m_s  * seconds;
    double s_max = ego.s + speed_m_s * (seconds+1);

    std::vector<char> occupied_lanes;
    occupied_lanes.push_back(' ');
    occupied_lanes.push_back(' ');
    occupied_lanes.push_back(' ');


    for(auto other_car: cars) {
      double other_car_s = other_car.s + other_car.speed * 0.44 * seconds;

      if(other_car_s > s_min && other_car_s < s_max) {
        long lane = long(other_car.d / 4);
        if (lane > -1 && lane < 3) {
          occupied_lanes[lane] = 'X';
        }
      }
    }
    maze.push_back(occupied_lanes);
  }

  for(int index = maze.size()-1; index >= 0; index--) {
    auto lane = maze[index];
    std::cout << "|" << lane[0] <<  "|" << lane[1] <<  "|" << lane[2] << "|" << " t = " << index << std::endl;
  }

}
