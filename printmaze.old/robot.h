// DriveToPoint(Point point);
// Windrichting

#ifndef PWS_ROBOT_H_
#define PWS_ROBOT_H_

#include "Initializable.h"
#include "point.h"
#include "hcsr04.h"
#include "tcs3200.h"
#include "l293d.h"

class Robot : public Initializable {
 public:
  Robot(L293D& motor_driver, TCS3200& color_sensor, HCSR04& left_distance_sensor, HCSR04& right_distance_sensor, HCSR04& front_distance_sensor);
  void Initialize() override;
  HCSR04& left_distance_sensor();
  HCSR04& right_distance_sensor();
  HCSR04& front_distance_sensor();
  L293D& motor_driver();
  
 private:
  L293D * motor_driver_;
  TCS3200 * color_sensor_;
  HCSR04 * left_distance_sensor_;
  HCSR04 * right_distance_sensor_;
  HCSR04 * front_distance_sensor_;
  Point current_location_;  
};

#endif
