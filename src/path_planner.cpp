#include "path_planner.h"
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include "helpers.h"
#include "coordinates_helper.h"
#include "vehicle.h"
#include "spline.h"
#include "json.hpp"

PathPlanner::PathPlanner() {
  vector<vector<Vehicle>> tmp(3, vector<Vehicle>(0));
  vehicles_in_lanes_ = tmp;
}

void PathPlanner::sense_environment(json sensor_fusion) {
  for (short i=0; i<NUMBER_OF_LANES; i++) {
    vehicles_in_lanes_[i].clear();
  }
  short object_lane;
  int id;
  double x, y, s, vx, vy, object_speed, object_yaw, d;
  Vehicle vehicle_in_sight;
  for (auto detected_object : sensor_fusion) {
    d = detected_object[6];
    s = detected_object[5];
    if (d < 0)
      continue;
    id = detected_object[0];
    x = detected_object[1];
    y = detected_object[2];
    vx = detected_object[3];
    vy = detected_object[4];
    object_speed = sqrt(pow(vx, 2) + pow(vy, 2));
    object_yaw = atan2(vy, vx);
    vehicle_in_sight = Vehicle(id, x, y, s, d, object_yaw, object_speed);
    object_lane = (short) (d / LANE_WIDTH);
    vehicles_in_lanes_[object_lane].push_back(vehicle_in_sight);
  }
}

bool PathPlanner::vehicle_ahead(Vehicle& ego_vehicle) {
  const unsigned int trajectory_size = previous_path_x.size();
  bool vehicle_ahead_ = false;
  for (auto vehicle_in_sight : vehicles_in_lanes_[target_lane]) {
    double s_hat = vehicle_in_sight.getS() + trajectory_size * UPDATE_RATE * vehicle_in_sight.getSpeed();
    if ((s_hat - ego_vehicle.getS()) < S_THRESHOLD && (s_hat - ego_vehicle.getS()) > 0) {
      vehicle_ahead_ = true;
    }
  }
  return vehicle_ahead_;
}

bool PathPlanner::loaded_lane(short lane, Vehicle& ego_vehicle) {
  const unsigned int trajectory_size = previous_path_x.size();
  bool loaded_lane_ = false;
  for (auto vehicle_in_sight : vehicles_in_lanes_[lane]) {
    double s_hat = vehicle_in_sight.getS() + trajectory_size * UPDATE_RATE * vehicle_in_sight.getSpeed();
    if ((ego_vehicle.getS() - S_THRESHOLD / 2 < s_hat) && (ego_vehicle.getS() + S_THRESHOLD > s_hat))
      loaded_lane_ = true;
  }
  return loaded_lane_;
}

void PathPlanner::plan_manoeuvre(Vehicle& ego_vehicle) {
  short current_lane = (short) (ego_vehicle.getD() / LANE_WIDTH);
  target_lane = current_lane;
  speed_diff = 0;

  if (vehicle_ahead(ego_vehicle)) {
    //  Analyse a change to the left or right lane
    if ((current_lane + 1 < NUMBER_OF_LANES) && !loaded_lane(current_lane + 1, ego_vehicle))
      target_lane = current_lane + 1;
    else if ((current_lane - 1 >= 0) && !loaded_lane(current_lane - 1, ego_vehicle))
      target_lane = current_lane - 1;
    else
      speed_diff -= ACC_MAX * 1;  // ACC_MAX * 1s
  }
  else {
    if (ego_vehicle.getSpeed() < SPEED_MAX)
      speed_diff += ACC_MAX;  // ACC_MAX * 1s
  }
}

void PathPlanner::init_path(Vehicle& ego_vehicle) {
  if (previous_path_x.size() < 2) {  // Estimates with no previous path
    path_x.push_back(ego_vehicle.getX() - cos(ego_vehicle.getYaw()));
    path_y.push_back(ego_vehicle.getY() - sin(ego_vehicle.getYaw()));
  }
  else {  // Otherwise, initialize with the path history
    unsigned int size = previous_path_x.size();
    ego_vehicle.setX(previous_path_x[size-1]);
    ego_vehicle.setY(previous_path_y[size-1]);
    double prev_x = previous_path_x[size-2];
    double prev_y = previous_path_y[size-2];

    ego_vehicle.setYaw(atan2(
      ego_vehicle.getY() - prev_y,
      ego_vehicle.getX() - prev_x
    ));

    path_x.push_back(prev_x);
    path_y.push_back(prev_y);
  }
}

void PathPlanner::generate_path_coord(
  Vehicle& ego_vehicle,
  vector<double> map_waypoints_s,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y
) {
  init_path(ego_vehicle);
  path_x.push_back(ego_vehicle.getX());
  path_y.push_back(ego_vehicle.getY());

  // Generate points farther away
  for (unsigned short i=1; i<4; i++) {
    vector<double> target_waypoints = getXY(
      ego_vehicle.getS() + DEFAULT_CAR_LONG * 8 * i,
      DEFAULT_CAR_WIDTH + LANE_WIDTH * target_lane,
      map_waypoints_s,
      map_waypoints_x,
      map_waypoints_y
    );
    path_x.push_back(target_waypoints[0]);
    path_y.push_back(target_waypoints[1]);
  }

  // Path coordinates according to the vehicle's position.
  for (unsigned int i = 0; i < path_x.size(); i++ ) {
    double delta_x = path_x[i] - ego_vehicle.getX();
    double delta_y = path_y[i] - ego_vehicle.getY();

    path_x[i] = delta_x * cos(-ego_vehicle.getYaw()) - delta_y * sin(-ego_vehicle.getYaw());
    path_y[i] = delta_x * sin(-ego_vehicle.getYaw()) + delta_y * cos(-ego_vehicle.getYaw());
  }
}

void PathPlanner::smooth_trajectory(Vehicle& ego_vehicle) {
  tk::spline s;
	s.set_points(path_x, path_y);

	// Init vector with latest points (for continuity)
	for (unsigned int i=0; i<previous_path_x.size(); i++) {
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	// Estimate the position DEFAULT_CAR_LONG * 8 ahead
	double target_x = DEFAULT_CAR_LONG * 8;
	double target_y = s(target_x);
	double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
	double offset_x = 0.;

	for (int i = previous_path_x.size(); i < NUMBER_OF_FITTED_POINTS; i++) {
    ego_vehicle.setSpeed(ego_vehicle.getSpeed() + speed_diff);
    if (ego_vehicle.getSpeed() > SPEED_MAX)
      ego_vehicle.setSpeed(SPEED_MAX);
    else if (ego_vehicle.getSpeed() < ACC_MAX)
      ego_vehicle.setSpeed(ACC_MAX);
    // Fit using the spline
    double N = target_dist/(UPDATE_RATE*ego_vehicle.getSpeed());
    double step = target_x / N;

		double x_hat = offset_x + step;
		double y_hat = s(x_hat);
		offset_x = x_hat;

    // From vehicle perspective to map perspective
		double x_tmp = x_hat;
		double y_tmp = y_hat;
		x_hat = (x_tmp * cos(ego_vehicle.getYaw()) - y_tmp*sin(ego_vehicle.getYaw()));
		y_hat = (x_tmp * sin(ego_vehicle.getYaw()) + y_tmp*cos(ego_vehicle.getYaw()));

		next_x_vals.push_back(x_hat + ego_vehicle.getX());
		next_y_vals.push_back(y_hat + ego_vehicle.getY());
	}
}

vector<vector<double>> PathPlanner::generate_trajectory(
  Vehicle& ego_vehicle,
  vector<double> map_waypoints_s,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y
) {
  next_x_vals.clear();
  next_y_vals.clear();
  path_x.clear();
  path_y.clear();

  generate_path_coord(ego_vehicle, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  smooth_trajectory(ego_vehicle);
  vector<vector<double>> next_vals;
  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);
  return next_vals;
}
