#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <vector>
#include "vehicle.h"
#include "json.hpp"
#include "helpers.h"

// for convenience
using std::string;
using std::vector;
using json = nlohmann::json;

const double UPDATE_RATE = 0.02;  // [s]
const double LANE_WIDTH = 4.0;  // [m]
const short NUMBER_OF_LANES = 3;
const double S_THRESHOLD = DEFAULT_CAR_LONG * 6;
const double ACC_MAX = 9 * UPDATE_RATE;  // 9[m/s2] * 0.02 [update_rate]
const double SPEED_MAX = 49 / MS2MPH;  // [m/s]
const int NUMBER_OF_FITTED_POINTS = 50;

class PathPlanner {
public:
  PathPlanner();
  ~PathPlanner() {};

  /**
   * Uses information about sensor fusion to detect nearby objects.
   * @param sensor_fusion : a json with of all other car's attributes on the same side of the road
   */
  void sense_environment(json sensor_fusion);
  /**
   * Based on the information of nearby objects, decide which action the ego-vehicle should take.
   * @param ego_vehicle
   */
  void plan_manoeuvre(Vehicle& ego_vehicle);
  /**
   * Generate a sequence of points that the ego-vehicle should follow based on the planned manoeuvre.
   * @param ego_vehicle
   * @param map_waypoints_s the s value is the distance along the road to get to the waypoints in meters
   * @param map_waypoints_x waypoints' map x coordinate position
   * @param map_waypoints_y waypoints' map y coordinate position
   */
  vector<vector<double>> generate_trajectory(
    Vehicle& ego_vehicle,
    vector<double> map_waypoints_s,
    vector<double> map_waypoints_x,
    vector<double> map_waypoints_y
  );

  json previous_path_x;
  json previous_path_y;
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  short target_lane;
  double speed_diff;
private:
  /**
   * Whether there's a vehicle at close distance
   * @param ego_vehicle
   */
  bool vehicle_ahead(Vehicle& ego_vehicle);
  /**
   * Whether the given lane contains vehicles avoiding the ego-vehicle to turn
   * @param lane the lane to check for vehicles
   * @param ego_vehicle
   */
  bool loaded_lane(short lane, Vehicle& ego_vehicle);
  /**
   * Init the trajectory with the previous path so that the resulting trajectory is smoother
   */
  void init_path(Vehicle& ego_vehicle);
  void generate_path_coord(
    Vehicle& ego_vehicle,
    vector<double> map_waypoints_s,
    vector<double> map_waypoints_x,
    vector<double> map_waypoints_y
  );
  void smooth_trajectory(Vehicle& ego_vehicle);

  vector<vector<Vehicle>> vehicles_in_lanes_;
  vector<double> path_x;
  vector<double> path_y;
};

#endif  // PATH_PLANNER_H_
