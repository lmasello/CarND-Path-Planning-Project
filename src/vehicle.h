#ifndef VEHICLE_H_
#define VEHICLE_H_

const double DEFAULT_CAR_WIDTH = 2.0;  // [m]
const double DEFAULT_CAR_LONG = 4.0;  // [m]

class Vehicle {
public:
  /**
   * Constructor initializes a default vehicle model.
   */
  Vehicle();
  /**
   * Constructor initializes a vehicle model with given params.
   * @param id
   * @param x : Cartesian's x coordinate [m]
   * @param y : Cartesian's y coordinate [m]
   * @param s : Frenet's s coordinate [m]
   * @param d : Frenet's d coordinate [m]
   * @param yaw : Yaw [rad]
   * @param speed : Velocity [m/s]
   */
  Vehicle(int id, double x, double y, double s, double d, double yaw, double speed);

  // Destructor
  ~Vehicle() {};

  int id;
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double width;
};

#endif  // VEHICLE_H_
