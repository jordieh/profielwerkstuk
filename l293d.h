#ifndef PWS_L93D_H_
#define PWS_L93D_H_

#include "Initializable.h"
#include "Arduino.h"

class L293D : public Initializable {
 public:
  L293D(int in_1, int in_2, int in_3, int in_4, int enable_1_2, int enable_3_4);
  void DriveForward();
  void Stop();
  void TurnLeft();
  void TurnRight();
  void DriveBackwards();
  void Initialize() override;
  void Enable12();
  void Disable12();
  void Enable34();
  void Disable34();
  void Lock();
  void Unlock();
  byte speed();
  void set_speed(byte speed);
  
 private:
  int in_1_;
  int in_2_;
  int in_3_;
  int in_4_;
  int enable_1_2_;
  int enable_3_4_;
  byte speed_;
};

#endif
