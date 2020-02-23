#include "l293d.h"

#include "Arduino.h"

L293D::L293D(int in_1, int in_2, int in_3, int in_4, int enable_1_2, int enable_3_4) {
  in_1_ = in_1;
  in_2_ = in_2;
  in_3_ = in_3;
  in_4_ = in_4;
  enable_1_2_ = enable_1_2;
  enable_3_4_ = enable_3_4;
  speed_ = 255;
}

byte L293D::speed() {
  return speed_;
}

void L293D::set_speed(byte speed) {
  speed_ = speed;
}

void L293D::Enable12() {
  digitalWrite(enable_1_2_, HIGH);
}

void L293D::Enable34() {
  digitalWrite(enable_3_4_, HIGH);
}

void L293D::Disable12() {
  digitalWrite(enable_1_2_, LOW);
}

void L293D::Disable34() {
  digitalWrite(enable_3_4_, LOW);
}

void L293D::Lock() {
//  Disable12();
//  Disable34();
}

void L293D::Unlock() {
//  Enable12();
//  Enable34();
}

void L293D::Initialize() {
  pinMode(in_1_, OUTPUT);
  pinMode(in_2_, OUTPUT);
  pinMode(in_3_, OUTPUT);
  pinMode(in_4_, OUTPUT);
  pinMode(enable_1_2_, OUTPUT);
  pinMode(enable_3_4_, OUTPUT);
}

void L293D::DriveForward() {
  analogWrite(in_1_, speed_);
  digitalWrite(in_2_, LOW);
  analogWrite(in_3_, speed_);
  digitalWrite(in_4_, LOW);
}

void L293D::TurnLeft() {
  digitalWrite(in_1_, LOW);
  digitalWrite(in_2_, HIGH);
  digitalWrite(in_3_, HIGH);
  digitalWrite(in_4_, LOW);
}

void L293D::TurnRight() {
  digitalWrite(in_1_, HIGH);
  digitalWrite(in_2_, LOW);
  digitalWrite(in_3_, LOW);
  digitalWrite(in_4_, HIGH);
}

void L293D::Stop() {
  digitalWrite(in_1_, LOW);
  digitalWrite(in_2_, LOW);
  digitalWrite(in_3_, LOW);
  digitalWrite(in_4_, LOW);
}

void L293D::DriveBackwards() {
  digitalWrite(in_1_, LOW);
  digitalWrite(in_2_, HIGH);
  digitalWrite(in_3_, LOW);
  digitalWrite(in_4_, HIGH);
}
