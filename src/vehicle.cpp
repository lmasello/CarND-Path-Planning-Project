#include "vehicle.h"

Vehicle::Vehicle() {
  id = -1;
  x = 0.;
  y = 0.;
  s = 0.;
  d = 0.;
  yaw = 0.;
  speed = 0.;
  width = DEFAULT_CAR_WIDTH;
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double speed) {
  id = id;
  x = x;
  y = y;
  s = s;
  d = d;
  yaw = yaw;
  speed = speed;
  width = DEFAULT_CAR_WIDTH;
}
