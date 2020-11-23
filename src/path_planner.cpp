#include "path_planner.h"

PathPlanner::PathPlanner(Vehicle ego_vehicle, double update_rate) {
  vector<vector<Vehicle>> tmp(3, vector<Vehicle>(1));
  vehicles_in_lanes_ = tmp;
  ego_vehicle = ego_vehicle;
  target_speed = ego_vehicle.speed;
  update_rate = update_rate;
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
    id = detected_object[0];
    x = detected_object[1];
    y = detected_object[2];
    vx = detected_object[3];
    vy = detected_object[4];
    object_speed = sqrt(pow(vx, 2) + pow(vy, 2));
    object_yaw = atan2(vy, vx);
    d = detected_object[6];
    s = detected_object[5];
    vehicle_in_sight = Vehicle(id, x, y, s, d, object_yaw, object_speed);
    object_lane = fmod(d, LANE_WIDTH);
    vehicles_in_lanes_[object_lane].push_back(vehicle_in_sight);
  }
}

bool PathPlanner::vehicle_ahead() {
  const unsigned int trajectory_size = previous_path_x.size();
  bool vehicle_ahead_ = false;
  for (auto vehicle_in_sight : vehicles_in_lanes_[target_lane]) {
    double s_hat = vehicle_in_sight.s + trajectory_size * update_rate;
    if ((s_hat - ego_vehicle.s) < S_THRESHOLD)
      vehicle_ahead_ = true;
  }
  return vehicle_ahead_;
}

bool PathPlanner::loaded_lane(short lane) {
  const unsigned int trajectory_size = previous_path_x.size();
  bool loaded_lane_ = false;
  for (auto vehicle_in_sight : vehicles_in_lanes_[lane]) {
    double s_hat = vehicle_in_sight.s + trajectory_size * update_rate;
    if ((ego_vehicle.s - S_THRESHOLD < s_hat) && (ego_vehicle.s + S_THRESHOLD > s_hat))
      loaded_lane_ = true;
  }
  return loaded_lane_;
}

void PathPlanner::plan_manoeuvre() {
  short current_lane = fmod(ego_vehicle.d, LANE_WIDTH);
  target_lane = current_lane;

  if (vehicle_ahead()) {
    //  Analyse a change to the left or right lane
    if ((current_lane + 1 < NUMBER_OF_LANES) && !loaded_lane(current_lane + 1))
      target_lane = current_lane + 1;
    else if ((current_lane - 1 >= 0) && !loaded_lane(current_lane - 1))
      target_lane = current_lane - 1;
    else
      target_speed -= ACC_MAX * 1;  // ACC_MAX * 1s
  }
  else {
    if (ego_vehicle.speed < SPEED_MAX)
      target_speed += ACC_MAX * 1;  // ACC_MAX * 1s
  }
}

void PathPlanner::init_path() {
  if (previous_path_x.size() < 2) {  // Estimates with no previous path
    path_x.push_back(ego_vehicle.x - cos(ego_vehicle.yaw) * ego_vehicle.speed * update_rate);
    path_y.push_back(ego_vehicle.y - sin(ego_vehicle.yaw) * ego_vehicle.speed * update_rate);
  }
  else {  // Otherwise, initialize with the path history
    ego_vehicle.x = previous_path_x[-1];
    ego_vehicle.y = previous_path_y[-1];
    double prev_y = previous_path_y[-2];
    double prev_x = previous_path_x[-2];

    ego_vehicle.yaw = atan2(
      prev_y / ego_vehicle.y,
      prev_x / ego_vehicle.x
    );

    path_x.push_back(prev_x);
    path_y.push_back(prev_y);
  }
}

void PathPlanner::generate_path_coord(vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
  init_path();
  path_x.push_back(ego_vehicle.x);
  path_y.push_back(ego_vehicle.y);

  // Generate points farther away
  for (unsigned short i=1; i<5; i++) {
    vector<double> target_waypoints = getXY(
      ego_vehicle.s + DEFAULT_CAR_LONG * 4 * i,
      ego_vehicle.width + LANE_WIDTH * target_lane,
      map_waypoints_s,
      map_waypoints_x,
      map_waypoints_y
    );
    path_x.push_back(target_waypoints[0]);
    path_y.push_back(target_waypoints[1]);
  }

  // Path coordinates according to the vehicle's position.
  for (unsigned int i = 0; i < path_x.size(); i++ ) {
    double delta_x = path_x[i] - ego_vehicle.x;
    double delta_y = path_y[i] - ego_vehicle.y;

    path_x[i] = delta_x * cos(-ego_vehicle.yaw) - delta_y * sin(-ego_vehicle.yaw);
    path_y[i] = delta_x * sin(-ego_vehicle.yaw) + delta_y * cos(-ego_vehicle.yaw);
  }
}

void PathPlanner::smooth_trajectory() {
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

	// Fit using the spline
  double N = target_dist/(update_rate*target_speed/MS2MPH);
  double step = target_x / N;
	for (int i = previous_path_x.size(); i < NUMBER_OF_FITTED_POINTS; i++) {
		double x_hat = offset_x + step;
		double y_hat = s(x_hat);
		offset_x = x_hat;

    // From vehicle perspective to map perspective
		double x_tmp = x_hat;
		double y_tmp = y_hat;
		x_hat = (x_tmp * cos(ego_vehicle.yaw) - y_tmp*sin(ego_vehicle.yaw));
		y_hat = (x_tmp * sin(ego_vehicle.yaw) + y_tmp*cos(ego_vehicle.yaw));

		next_x_vals.push_back(x_hat + ego_vehicle.x);
		next_y_vals.push_back(y_hat + ego_vehicle.y);
	}
}

vector<vector<double>> PathPlanner::generate_trajectory(
  vector<double> map_waypoints_s,
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y
) {
  next_x_vals.clear();
  next_y_vals.clear();
  path_x.clear();
  path_y.clear();

  generate_path_coord(map_waypoints_s, map_waypoints_x, map_waypoints_y);
  smooth_trajectory();
  vector<vector<double>> next_vals;
  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);
  return next_vals;
}
