#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <math.h>
#include <string>
#include <vector>
#include "helpers.h"
#include "vehicle.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using std::string;
using std::vector;
using json = nlohmann::json;

const double LANE_WIDTH = 4.0;  // [m]
const short NUMBER_OF_LANES = 3;
const double S_THRESHOLD = DEFAULT_CAR_LONG * 8;
const double ACC_MAX = 0.2;  // [m/s2]
const double SPEED_MAX = 50;  // [m/s]
const int NUMBER_OF_FITTED_POINTS = 50;

class PathPlanner {
public:
  PathPlanner(Vehicle ego_vehicle, double update_rate);
  ~PathPlanner() {};

  void sense_environment(json sensor_fusion);
  void plan_manoeuvre();
  vector<vector<double>> generate_trajectory(
    vector<double> map_waypoints_s,
    vector<double> map_waypoints_x,
    vector<double> map_waypoints_y
  );

  json previous_path_x;
  json previous_path_y;
  vector<double> next_x_vals;
  vector<double> next_y_vals;

private:
  bool vehicle_ahead();
  bool loaded_lane(short lane);
  void init_path();
  void generate_path_coord(
    vector<double> map_waypoints_s,
    vector<double> map_waypoints_x,
    vector<double> map_waypoints_y
  );
  void smooth_trajectory();

  vector<vector<Vehicle>> vehicles_in_lanes_;
  Vehicle ego_vehicle;
  short target_lane;
  double target_speed;
  vector<double> path_x;
  vector<double> path_y;
  double update_rate;
};

#endif  // PATH_PLANNER_H_
