//#include "maze.h"
//
//Maze::Maze(Size size) {
//  this->size = size;
//  this->matrix = std::vector<byte>(size.width * size.height);
//}
//
//Size Maze::getSize() {
//  return this->size;
//}
//
//byte & Maze::operator[](Position position) {
//  // Because we cannot define the size of 2d arrays at runtime in C++ we need to use Row-major ordering (https://en.wikipedia.org/wiki/Row-_and_column-major_order)
//  return matrix[(size.height - 1 - position.y) * size.width + position.x];
//}
