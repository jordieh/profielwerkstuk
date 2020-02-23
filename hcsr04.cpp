#include "hcsr04.h"

#include "Arduino.h"

HCSR04::HCSR04(int trig, int echo) {
  trig_ = trig;
  echo_ = echo;
}

void HCSR04::set_cycles(unsigned int cycles) {
  cycles_ = cycles;
}

double HCSR04::offset() {
  return offset_;
}

void HCSR04::set_offset(double offset) {
  offset_ = offset;
}

unsigned int HCSR04::cycles() {
  return cycles_;
}

double HCSR04::Pulse() {
  if (cycles_ <= 1) {
    return PulseOnce();
  }

  double sum = 0;
  for (int i = 0; i < cycles_; i++) {
    sum += PulseOnce();
    delay(5);
  }
  double result = sum / cycles_;
  return result <= 0 ? INVALID_DISTANCE : result + offset_;
}

double HCSR04::PulseOnce() {
  // Retry up to 3 times in the case of an error
  static int pulse = 0;
  if (++pulse > 3) {
    pulse = 0;
    return INVALID_DISTANCE;
  }
  
  digitalWrite(trig_, LOW);
  delayMicroseconds(10);
  
  digitalWrite(trig_, HIGH);
  digitalWrite(trig_, LOW);

  unsigned long duration = pulseIn(echo_, HIGH, 2000);
  if (duration == 0) {
    return PulseOnce();
  } else {
    pulse = 0;
  }
  
  double distance = duration * 0.034 / 2;
  return distance; // cm
}

void HCSR04::Initialize() {
  pinMode(trig_, OUTPUT);
  pinMode(echo_, INPUT);
}
