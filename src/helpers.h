#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
inline string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// For converting back and forth speed values
const double MS2MPH = 2.23694;
inline double mph2ms(double x) { return x / MS2MPH; }
inline double ms2mph(double x) { return x * MS2MPH; }

// Calculate distance between two points
inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// // Calculate closest waypoint to current x, y position
// int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
//                     const vector<double> &maps_y) {
//   double closestLen = 100000; //large number
//   int closestWaypoint = 0;
//
//   for (int i = 0; i < maps_x.size(); ++i) {
//     double map_x = maps_x[i];
//     double map_y = maps_y[i];
//     double dist = distance(x,y,map_x,map_y);
//     if (dist < closestLen) {
//       closestLen = dist;
//       closestWaypoint = i;
//     }
//   }
//
//   return closestWaypoint;
// }
//
// // Returns next waypoint of the closest waypoint
// int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
//                  const vector<double> &maps_y) {
//   int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
//
//   double map_x = maps_x[closestWaypoint];
//   double map_y = maps_y[closestWaypoint];
//
//   double heading = atan2((map_y-y),(map_x-x));
//
//   double angle = fabs(theta-heading);
//   angle = std::min(2*pi() - angle, angle);
//
//   if (angle > pi()/2) {
//     ++closestWaypoint;
//     if (closestWaypoint == maps_x.size()) {
//       closestWaypoint = 0;
//     }
//   }
//
//   return closestWaypoint;
// }


#endif  // HELPERS_H
