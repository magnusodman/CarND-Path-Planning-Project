//
// Created by Magnus Ödman on 2017-09-09.
//

#include <iostream>
#include "pathplanner.h"
#include "AStar.h"

STATE pathplanner::PlanPath(car ego, std::vector<car> cars) {

  double dt = 1.0;

  double speed_m_s = ego.speed * .447;


  std::vector<std::vector<char>> maze;
  //Check 12 seconds into the future
  for (int iteration = 0; iteration < 8; iteration++) {
    int seconds = iteration * 2;
    double s_min = -10.0 + ego.s + speed_m_s * seconds;
    double s_max = ego.s + speed_m_s * (seconds + 2);

    std::vector<char> occupied_lanes;
    occupied_lanes.push_back(' ');
    occupied_lanes.push_back(' ');
    occupied_lanes.push_back(' ');


    for (auto other_car: cars) {

      double other_car_s = other_car.s + other_car.speed * 0.44 * seconds;

      if (other_car_s > s_min && other_car_s < s_max) {
        long lane = long(other_car.d / 4);
        if (lane > -1 && lane < 3) {
          occupied_lanes[lane] = 'X';
        }
      }
    }
    maze.push_back(occupied_lanes);
  }

  long ego_lane = long(ego.d / 4);
  if (ego_lane >= 0 && ego_lane < 3) {
    maze[0][ego_lane] = 'H';
    maze[maze.size() - 1][ego_lane] = 'O';
  }






  //Search path
  //https://github.com/daancode/a-star

  AStar::Generator generator;
  // Set 2d map size.
  generator.setWorldSize({int(maze.size()), 3});
  // You can use a few heuristics : manhattan, euclidean or octagonal.
  generator.setHeuristic(AStar::Heuristic::euclidean);
  generator.setDiagonalMovement(false);

  for (int row = 0; row < maze.size(); row++) {
    for (int col = 0; col < 3; col++) {
      if (maze[row][col] == 'X') {
        generator.addCollision({row, col});

      }
    }

  }

  // This method returns vector of coordinates from target to source.
  auto path = generator.findPath({0, int(ego_lane)}, {int(maze.size()) - 1, int(ego_lane)});

  for (int coordinate_index = 0; coordinate_index < path.size(); coordinate_index++) {
    auto coordinate = path[coordinate_index];
    int row = coordinate.x;
    int col = coordinate.y;
    if (maze[row][col] != 'H' && maze[row][col] != 'O') {
      if (maze[row][col] == 'X') {
        maze[row][col] = '*';
      } else {
        maze[row][col] = std::to_string(coordinate_index)[0];
      }
    }

  }

  STATE state = KEEP_LANE;
  auto next = path[path.size() - 2];
  if (path.size() > 2) {
    if (next.x == 1) {
      state = KEEP_LANE;
    } else {
      if (next.y > ego_lane) {
        state = CHANGE_LANE_RIGHT;
      }
      if (next.y < ego_lane) {
        state = CHANGE_LANE_LEFT;
      }
    }
  }

  char signal = ' ';
  if (state == CHANGE_LANE_LEFT) {
    signal = '<';
  }
  if (state == CHANGE_LANE_RIGHT) {
    signal = '>';
  }

  for (int index = maze.size() - 1; index >= 0; index--) {
    auto lane = maze[index];
    std::cout << "|" << lane[0] << "|" << lane[1] << "|" << lane[2] << "|" << " t = " << index << " " << signal
              << std::endl;
  }
  std::cout << std::endl;
  return state;
}
