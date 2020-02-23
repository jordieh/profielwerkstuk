#ifndef PWS_TCS3200_H_
#define PWS_TCS3200_H_

#include "initializable.h"

enum Color {
  RED = 1, // 0 = NULL
  LIGHT_RED,
  BROWN,
  BLACK,
  GREEN,
  LIGHT_BLUE,
  BLUE
};

class TCS3200 : public Initializable {
 public:
  TCS3200(int s0, int s1, int s2, int s3, int out);
  Color Read();
  void Initialize() override;
  
 private:
  int s0_;
  int s1_;
  int s2_;
  int s3_;
  int out_;
};

#endif
