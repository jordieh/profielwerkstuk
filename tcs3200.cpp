#include "tcs3200.h"

#include "Arduino.h"

TCS3200::TCS3200(int s0, int s1, int s2, int s3, int out) {
  s0_ = s0;
  s1_ = s1;
  s2_ = s2;
  s3_ = s3;
  out_ = out;
}

void TCS3200::Initialize() {
  pinMode(s0_, OUTPUT);
  pinMode(s1_, OUTPUT);
  pinMode(s2_, OUTPUT);
  pinMode(s3_, OUTPUT);
  pinMode(out_, INPUT);

  digitalWrite(s0_, HIGH);
  digitalWrite(s1_, LOW);
}

// https://randomnerdtutorials.com/arduino-color-sensor-tcs230-tcs3200/
Color TCS3200::Read() {
//        Serial.flush();
//        delay(100);
  digitalWrite(s2_, LOW);
  digitalWrite(s3_, LOW);
  int r = pulseIn(out_, LOW);
  Serial.print("R = ");
  Serial.print(r);
  delay(100);

  digitalWrite(s2_, HIGH);
  digitalWrite(s3_, HIGH);
  int g = pulseIn(out_, LOW);
  Serial.print(" G = ");
  Serial.print(g);
  delay(100);

  digitalWrite(s2_, LOW);
  digitalWrite(s3_, HIGH);
  int b = pulseIn(out_, LOW);
  Serial.print(" B = ");
  Serial.print(b);
  delay(100);

  Color result;
  if (r >= 20 && r <= 35 && g >= 30 && g <= 48 && b >= 28 && b <= 44) {
    result = Color::LIGHT_RED;
  } else if (r >= 24 && r <= 39 && g >= 24 && g <= 39 && b >= 20 && b <= 32) {
    result = Color::LIGHT_BLUE;
  } else if (r >= 28 && r <= 50 && g >= 109 && g <= 156 && b >= 81 && b <= 115) {
    result = Color::RED;
  } else if (r >= 53 && r <= 64 && g >= 98 && g <= 119 && b >= 99 && b <= 123) {
    result = Color::BROWN;
  } else if (r >= 170 && r <= 180 && g >= 246 && g <= 253 && b >= 202 && b <= 211) {
    result = Color::BLACK;
  } else if (r >= 29 && r <= 32 && g >= 36 && g <= 39 && b >= 48 && b <= 56) {
    result = Color::GREEN;
  } else if (r >= 128 && r <= 143 && g >= 86 && g <= 99 && b >= 50 && b <= 67) {
    result = Color::BLUE;
  } else {
    result = NULL;
  }

  Serial.print(" => color = ");
  Serial.println(result);
  
  return result;
}
