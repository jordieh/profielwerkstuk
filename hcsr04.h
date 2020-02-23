#ifndef PWS_HCSR04_H_
#define PWS_HCSR04_H_

#define INVALID_DISTANCE -1

#include "initializable.h"

class HCSR04 : public Initializable {
 public:
  HCSR04(int trig, int echo);
  double Pulse();
  double PulseOnce();
  void Initialize() override;
  void set_cycles(unsigned int cycles);
  unsigned int cycles();
  double offset();
  void set_offset(double offset);
  
 private:
  int trig_;
  int echo_;
  unsigned int cycles_;
  double offset_;
};

#endif
