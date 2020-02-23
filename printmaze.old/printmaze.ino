//// Required to load our libraries.
////#define BOOST_NO_CXX11_HDR_TYPEINDEX
////#define BOOST_NO_CXX11_HDR_TUPLE
////#define BOOST_NO_CXX11_HDR_ARRAY
//#define BOOST_NO_CWCHAR
//#include <ArduinoSTL.h>
////#include <boost_1_51_0.h>
//
//// Library headers
////#include <boost/graph/adjacency_matrix.hpp>
////#include <boost/graph/graph_utility.hpp>
//
//// Our headers
//#include "Point.h"
//#include "Edge.h"
//#include "hcsr04.h"
//#include "l293d.h"
//#include "tcs3200.h"
//#include "Maze.h"
//
//// Undirected graph with (x, y) coordinates as vertexes.
////typedef boost::adjacency_matrix<boost::undirectedS, Point> Graph;
//
////Maze maze({.width = 4, .height = 4});
//
//// Pin mappings are present in the schematic.
//// Hardware
//HCSR04 left_distance_sensor(13, 4);
//HCSR04 right_distance_sensor(13, 5);
//HCSR04 front_distance_sensor(13, 3);
//L293D motor_driver(11, 10, 9, 6, 7, 8);
//TCS3200 color_sensor(A0, A1, A2, A3, A4);
//
//const int distance_sensor_cycles = 5;
//
//// Maze
////Graph graph(4 * 4); // We need sufficient space in memory to store our maze.
//Point start_location(0, 0);
//Point end_location; // Needs to be found. (Red spot on the ground)
//Point current_location;
//boolean finished;
//
//void setup() {
//  Serial.begin(9600); // Initialize communication channel with computer.
//  left_distance_sensor.Initialize();
//  left_distance_sensor.set_cycles(distance_sensor_cycles);
////  left_distance_sensor.set_offset(-2.78);
//  right_distance_sensor.Initialize();
//  right_distance_sensor.set_cycles(distance_sensor_cycles);
//  front_distance_sensor.Initialize();
//  front_distance_sensor.set_cycles(distance_sensor_cycles);
//  color_sensor.Initialize();
//  motor_driver.Initialize();
//  
//  current_location = start_location;
//  for (int i = 0; i < 500; i++) { 
//  Serial.println(right_distance_sensor.Pulse());
//   }
//}
//
//void loop() {
////  if (finished) {
////    return;
////  }
////
////  if (color_sensor.Read() == Color::RED) {
////    end_location = current_location;
////    return; // ?
////  }
//
////  Serial.print("");
////  Serial.print(left_distance_sensor.Pulse());
////  Serial.print(" ");
////  Serial.print(" ");
////  Serial.println(front_distance_sensor.Pulse());
////  delay(100);
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
////#define SIMPLE_PRINT_FUNCTION // printSimpleMazeToSerialMonitor();
////#define PRETTY_PRINT_FUNCTION // printFullyDecoratedMazeToSerialMonitor();
//
////void setup4by4Maze() {
////  maze[{0, 0}] = NORTH_NEIGHBOUR | EAST_NEIGHBOUR;
////  maze[{1, 0}] = NORTH_NEIGHBOUR | WEST_NEIGHBOUR;
////  maze[{2, 0}] = NORTH_NEIGHBOUR | EAST_NEIGHBOUR;
////  maze[{3, 0}] = NORTH_NEIGHBOUR | WEST_NEIGHBOUR;
////
////  maze[{0, 1}] = START_POSITION | SOUTH_NEIGHBOUR;
////  maze[{1, 1}] = NORTH_NEIGHBOUR | EAST_NEIGHBOUR | SOUTH_NEIGHBOUR;
////  maze[{2, 1}] = SOUTH_NEIGHBOUR | WEST_NEIGHBOUR;
////  maze[{3, 1}] = NORTH_NEIGHBOUR | SOUTH_NEIGHBOUR;
////
////  maze[{0, 2}] = NORTH_NEIGHBOUR;
////  maze[{1, 2}] = NORTH_NEIGHBOUR | SOUTH_NEIGHBOUR;
////  maze[{2, 2}] = NORTH_NEIGHBOUR | EAST_NEIGHBOUR;
////  maze[{3, 2}] = SOUTH_NEIGHBOUR | WEST_NEIGHBOUR;
////
////  maze[{0, 3}] = SOUTH_NEIGHBOUR | EAST_NEIGHBOUR;
////  maze[{1, 3}] = WEST_NEIGHBOUR | SOUTH_NEIGHBOUR;
////  maze[{2, 3}] = SOUTH_NEIGHBOUR | EAST_NEIGHBOUR;
////  maze[{3, 3}] = END_POSITION | WEST_NEIGHBOUR;
////}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//// The code below is awfull and you should not look at it.
//// It is purely written to print out the maze, nothing is modified.
//
//#ifdef PRETTY_PRINT_FUNCTION
//void printFullyDecoratedMazeToSerialMonitor() {
//  Size size = maze.getSize();
//
//  String nextLine = "";
//  String endRowBuffer = "";
//  for (int y = size.height - 1; y >= 0; y--) {
//    for (int x = 0; x < size.width; x++) {
//      byte val = maze[{x, y}];
//      if (y == size.height - 1) {
//        if (x == 0) {
//          Serial.print("┌──");
//        } else if (x == size.width - 1) {
//          Serial.print("━─┐");
//        } else if (maze[{x - 1, y}] & EAST_NEIGHBOUR) {
//          Serial.print("───┬");
//        } else {
//          Serial.print("───");
//        }
//      } else {
//        if (x == 0) {
//          if (val & NORTH_NEIGHBOUR && (maze[{x, y + 1}] & SOUTH_NEIGHBOUR)) {
//            Serial.print("│");
//          } else {
//            Serial.print("├");
//          }
//        }
//
//        if (val & NORTH_NEIGHBOUR && (maze[{x, y + 1}] & SOUTH_NEIGHBOUR)) {
//          Serial.print("  ");
//        } else {
//          Serial.print("──");
//        }
//
//        if (x == size.width - 1) {
//          if (val & NORTH_NEIGHBOUR && (maze[{x, y + 1}] & SOUTH_NEIGHBOUR)) {
//            Serial.print(" │");
//          } else {
//            Serial.print("┤");
//          }
//        } else {
//          if (val & NORTH_NEIGHBOUR & SOUTH_NEIGHBOUR & WEST_NEIGHBOUR & EAST_NEIGHBOUR) {
//            Serial.print(" ┼");
//          } else if (val & SOUTH_NEIGHBOUR && maze[{x + 1, y}] & SOUTH_NEIGHBOUR) {
//            if (val & NORTH_NEIGHBOUR) {
//              if (maze[{x, y - 1}] & EAST_NEIGHBOUR) {
//                Serial.print(" ::");
//              } else {
//                Serial.print(" ├"); 
//              }
//            } else {
//              if (val & NORTH_NEIGHBOUR) {
//                if (val & EAST_NEIGHBOUR) {
//                  if (val & SOUTH_NEIGHBOUR) {
//                    Serial.print(" FF"); 
//                  } else {
//                    Serial.print(" ZZ"); 
//                  }
//                } else {
//                  Serial.print(" RR"); 
//                }
//              } else {
//                if (val & EAST_NEIGHBOUR) {
//                  Serial.print(" BB");
//                } else {
//                  if (maze[{x - 1, y}] & NORTH_NEIGHBOUR) {
//                    Serial.print("┤");
//                  } else {
//                    Serial.print("┤");
//                  }
//                }
//              }
//            }
//          } else {
//            if (maze[{x - 1, y - 1}] & EAST_NEIGHBOUR) {
//              Serial.print(" ┌"); 
//            } else {
//              Serial.print(" │"); 
//            }
//          }
//        }
//      }
//
//      if (x == 0) {
//        nextLine += "│";
//      }
//      
//      nextLine += val & START_POSITION ? " S " : val & END_POSITION ? " E " : val & VISITED ? " V " : " X ";
//      if (val & EAST_NEIGHBOUR) {
//        nextLine += "  ";
//      } else if (x == size.width - 1) {
//        nextLine += "│";
//      } else {
//        nextLine += "│";
//      }
//
//      if (x == size.width - 1) {
//        Serial.println();
//        Serial.println(nextLine);
//        nextLine = "";
//      }
//    } // for (x)
//    
//    if (y == 0) {
//      for (int x = 0; x < size.width; x++) {
//          if (x == 0) {
//            Serial.print("└──");
//          } else if (x == size.width - 1) {
//            Serial.print("━─┘");
//          } else if (maze[{x - 1, y}] & EAST_NEIGHBOUR) {
//            Serial.print("───┴");
//          } else {
//            Serial.print("───");
//          }
//      }
//    } // y == 0
//    
//  }
//}
//#endif
//
//#ifdef SIMPLE_PRINT_FUNCTION
//void printSimpleMazeToSerialMonitor() {
//  Size size = maze.getSize();
//  Serial.print("Maze Size: ");
//  Serial.print(size.height);
//  Serial.print("x");
//  Serial.println(size.width);
//  for (int y = size.height - 1; y >= 0; y--) {
//    Serial.print("[");
//    for (int x = 0; x < size.width; x++) {
//      byte val = maze[{x, y}];
//      Serial.print(val, BIN);
//      if (x != size.width - 1) {
//         Serial.print(", ");
//      }
//    }
//    Serial.println("]");
//  }
//}
//#endif
