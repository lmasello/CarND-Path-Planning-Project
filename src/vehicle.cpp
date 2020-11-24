#include "vehicle.h"

Vehicle::Vehicle() {
  id_ = -1;
  x_ = 0.;
  y_ = 0.;
  s_ = 0.;
  d_ = 0.;
  yaw_ = 0.;
  speed_ = 0.;
  width_ = DEFAULT_CAR_WIDTH;
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double speed) {
  id_ = id;
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  yaw_ = yaw;
  speed_ = speed;
  width_ = DEFAULT_CAR_WIDTH;
}
