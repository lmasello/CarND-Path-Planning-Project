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

  double getX() {  return x_; }
  double getY() {  return y_; }
  double getS() {  return s_; }
  double getD() {  return d_; }
  double getYaw() {  return yaw_; }
  double getSpeed() {  return speed_; }
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setS(double s) { s_ = s; }
  void setD(double d) { d_ = d; }
  void setYaw(double yaw) { yaw_ = yaw; }
  void setSpeed(double speed) { speed_ = speed; }

private:
  int id_;
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;
  double width_;
};

#endif  // VEHICLE_H_
