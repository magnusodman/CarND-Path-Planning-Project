//
// Created by Magnus Ödman on 2017-09-10.
//

#include "trajectorygenerator.h"
#include <iostream>
#include "spline.h"

constexpr double l_pi() { return M_PI; }

double l_deg2rad(double x) { return x * l_pi() / 180; }

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double>
l_getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x, std::vector<double> maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - l_pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}

std::tuple<std::vector<double>, std::vector<double>>
trajectorygenerator::GenerateTrajectory(car ego, statemachine sm, std::vector<double> previous_path_x,
                                        std::vector<double> previous_path_y, std::vector<double> map_waypoints_s,
                                        std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y) {
  int prev_size = previous_path_x.size();


  //-----------------

  std::vector<double> ptsx;
  std::vector<double> ptsy;

  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = l_deg2rad(ego.yaw);

  if (prev_size < 2) {
    double prev_car_x = ego.x - cos(ego.yaw);
    double prev_car_y = ego.y - sin(ego.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ego.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ego.y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  double wp_distance = std::max(10.0, ego.speed * 0.44 * 2.5);
  for (int wp_index = 0; wp_index < 3; wp_index++) {

    std::vector<double> next_wp = l_getXY(ego.s + (wp_index + 1) * wp_distance, (2 + 4 * sm.lane), map_waypoints_s,
                                          map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);


  }

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

  }

  tk::spline s;
  s.set_points(ptsx, ptsy);


  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  for (int i = 0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;

  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

  double x_add_on = 0;

  for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
    double N = (target_dist / (.02 * sm.ref_vel / 2.24));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return std::make_tuple(next_x_vals, next_y_vals);
}
