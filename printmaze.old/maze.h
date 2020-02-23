//#include <ArduinoSTL.h>
//
//#include <vector>
//
//#include "Arduino.h"
//
//#include "size.h"
//#include "position.h"
//
//// Flags (https://en.wikipedia.org/wiki/Bit_field). Please note that we are not using the Arduino library bit manipulation functions.
//#define VISITED           0b10000000
//#define NORTH_NEIGHBOUR   0b01000000
//#define EAST_NEIGHBOUR    0b00100000
//#define SOUTH_NEIGHBOUR   0b00010000
//#define WEST_NEIGHBOUR    0b00001000
//#define START_POSITION    0b00000100
//#define END_POSITION      0b00000010
//
//class Maze {
//  public:
//    Maze(Size size);
//    byte & operator[] (Position position);
//    Size getSize();
//  private:
//    Size size;
//    std::vector<byte> matrix;
//};
