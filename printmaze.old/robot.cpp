#include "robot.h"

Robot::Robot(L293D& motor_driver, TCS3200& color_sensor, HCSR04& left_distance_sensor, HCSR04& right_distance_sensor, HCSR04& front_distance_sensor) {
  motor_driver_ = &motor_driver;
  color_sensor_ = &color_sensor;
  left_distance_sensor_ = &left_distance_sensor;
  right_distance_sensor_ = &right_distance_sensor;
  front_distance_sensor_ = &front_distance_sensor;
}

void Robot::Initialize() {
  left_distance_sensor_->Initialize();
  right_distance_sensor_->Initialize();
  front_distance_sensor_->Initialize();
  motor_driver_->Initialize();
  color_sensor_->Initialize();
}

HCSR04& Robot::front_distance_sensor() {
  return *front_distance_sensor_;
}

HCSR04& Robot::right_distance_sensor() {
  return *right_distance_sensor_;
}

HCSR04& Robot::left_distance_sensor() {
  return *left_distance_sensor_;
}

L293D& Robot::motor_driver() {
  return *motor_driver_;
}
