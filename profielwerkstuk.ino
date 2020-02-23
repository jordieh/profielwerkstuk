/**
 * Arduino versie: 1.8.9
 * 
 * Pin mapping
 * 
 * Top
 * +-------------+---------+--------+------+---------+------+---------+
 * | Pin board   | 43      | 44     | 45   | 46      | 47   | 48      |
 * | Pin Arduino | 13      | 3      | 13   | 4       | 13   | 2       |
 * | Function    | TRIG   | ECHO_FD | TRIG | ECHO_LD | TRIG | ECHO_RD |
 * +-------------+---------+--------+------+---------+------+---------+
 * 
 * Center
 * +-------------+-----+----+----+----+----+
 * | Pin board   | 29  | 30 | 31 | 32 | 33 |
 * | Pin Arduino | A4  | A3 | A2 | A1 | A0 |
 * | Function    | OUT | S0 | S1 | S2 | S3 |
 * +-------------+-----+----+----+----+----+
 * 
 * Bottom
 * +-------------+---------+---------+---------+---+----+------+------+----+----+----+----+------+----+----+----+----+----+----+
 * | Pin board   | 2       | 3       | 4       | 5 | 6  | 7    | 8    | 9  | 10 | 11 | 12 | 13   | 14 | 15 | 16 | 17 | 18 | 19 |
 * | Pin Arduino | 3       | 4       | 2       |   | 7  | 8    | 9    | 10 | 11 | 12 |    | 13   |    |    |    |    |    |    |
 * | Function    | ECHO_FD | ECHO_LD | ECHO_RD |   | LT | EN## | EN## | LB | RB | RT |    | TRIG |    |    |    |    |    |    |
 * +-------------+---------+---------+---------+---+----+------+------+----+----+----+----+------+----+----+----+----+----+----+
 */

/**
 * Library configuration
 */
#define BOOST_NO_CXX11_HDR_TYPEINDEX
#define BOOST_NO_CXX11_HDR_TUPLE
#define BOOST_NO_CXX11_HDR_ARRAY
#define BOOST_NO_CWCHAR
#define BOOST_NO_EXCEPTIONS

#include <ArduinoSTL.h>
#include <boost_1_51_0.h>

#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <utility> // std::pair
#include <vector> // std::vector
#include <algorithm> // std::reverse
#include <iterator> // std::distance

#include "hcsr04.h"
#include "tcs3200.h"
#include "l293d.h"

#define SERIAL_DEBUG_PWS

/**
 * Type definitions
 */
typedef std::pair<int, int> Point;
typedef boost::property<boost::edge_weight_t, int> WeightProperty;
typedef boost::adjacency_matrix<boost::undirectedS, Point, WeightProperty> MazeGraph;
typedef boost::graph_traits<MazeGraph>::vertex_descriptor vertex_descriptor;

enum class State { EXPLORING, FOUND_END, FOUND_PATH, FINISHED };
enum Direction {
  N = 1, // Make sure N != 0 so N != NULL
  E,
  S,
  W
};
const unsigned int maze_size = 4;

/**
 * Variable declarations
 */
HCSR04 lds(13, 4); // Left distance sensor
HCSR04 rds(13, 2); // Right distance sensor
HCSR04 fds(13, 3); // Front distance sensor
L293D driver(10, 7, 12, 11, 8, 9); // L293D motor driver
TCS3200 tcs(A3, A2, A1, A0, A4); // TCS3200 color sensor

MazeGraph graph(maze_size * maze_size);
std::vector<Point> s_path(16);

int error = 0;
Point cp;
Point start;
Point finish;
Direction cd = Direction::N;
State state = State::EXPLORING;

void setup() {
  Serial.begin(9600);
  
  driver.Initialize();
  tcs.Initialize();
  lds.Initialize();
  rds.Initialize();
  fds.Initialize();
  
  lds.set_cycles(5);
  lds.set_offset(0.442);
  rds.set_cycles(10);
  rds.set_offset(-0.2);
  fds.set_cycles(7);
  fds.set_offset(0.413);
  
  driver.Enable12();
  driver.Enable34();

  cp = {0, 0};
  start = cp;

  for (int x = 0; x < maze_size; x++) {
    for (int y = 0; y < maze_size; y++) {
      auto vd = boost::vertex(flatten_point(Point{x, y}), graph);
      graph[vd].first = x;
      graph[vd].second = y;
    }
  }

  debug("Initialization finished.");

  print_graph();
}

bool is_edge_in_shortest_path(const Point& source, const Point& target) {
  const auto limit = s_path.size();
  auto i = limit;
  while (i-- > 0) {
    if (s_path[i] == source && i > 0 && s_path[i - 1] == target || i < limit && s_path[i + 1] == target) {
      return true;
    }
  }
  return false;
}

void activate_edge(const Point& from, const Point& to) {
  auto vdf = boost::vertex(flatten_point(from), graph);
  auto vdt = boost::vertex(flatten_point(to), graph);
  auto out = boost::add_edge(vdf, vdt, graph);
  get(boost::edge_weight, graph)[out.first] = sqrt(sq(from.first - to.first) + sq(from.second - to.second));
}

// Converts a 2D cartesian point to a unique index.
int flatten_point(const Point& point) {
  return point.first + point.second * maze_size; // x + y * size
}

bool drive_to(const Point& to) {
  Direction driving_dir = get_direction(cp, to);
  if (!driving_dir) {
    return false; // Diagonal movement not supported or our target point is our current point.
  }
  
  rotate_to(driving_dir);
  while (cp != to) {
    drive_fwd_one();
  }
  return true;
}

inline void debug(String s) {
  #ifdef SERIAL_DEBUG_PWS
  Serial.println(s);
  #endif
}

inline void debug(int i) {
  #ifdef SERIAL_DEBUG_PWS
  Serial.println(i);
  #endif
}

void rotate_to(Direction d) {
  while (cd != d) {
    debug("Rotating right.");
    driver.TurnRight();
    delay(500); // Time to turn 90*
    driver.Stop();
    cd = next_dir_r(cd);
  }
}

void rot_fwd_empty() {
  Direction opposite_dir = next_dir_r(next_dir_r(cd));
  int count = 0;
  while(fds.Pulse() != INVALID_DISTANCE) {
    debug("No empty cell ahead, rotating!");
    top:
    rotate_to(next_dir_r(cd));
    if (cd == opposite_dir && count++ < 1) {
      debug("This is our counter direction and first circle, retry.");
      goto top;
    }
  }
}

Direction next_dir_r(Direction d) {
  switch (d) {
    case Direction::N:
      return Direction::E;
    case Direction::E:
      return Direction::S;
    case Direction::S:
      return Direction::W;
    case Direction::W:
      return Direction::N;
  }
}

/*
 * {0, 0}, {0, 1}  -> N
 * {0, 0}, {1, 0}  -> E
 * {0, 0}, {0, -1} -> S
 * {0, 0}, {-1, 0} -> W
 * {x, x}, {x, x}  -> NULL (same point)
 * {0, 0}, {1, 1}  -> NULL (diagonal)
 */
Direction get_direction(const Point& one, const Point& two) {
  if (one.second == two.second) {
    if (one.first > two.first) {
      return Direction::W;
    } else if (one.first < two.first) {
      return Direction::E;
    }
  } else if (one.first == two.first) {
    if (one.second > two.second) {
      return Direction::S;
    } else if (one.second < two.second) {
      return Direction::N;
    }
  }
  return NULL;
}

// Increments the point by one in the specified direction.
Point incr_point(const Point& point, Direction d) {
  switch (d) {
    case Direction::N:
      return {point.first, point.second + 1};
    case Direction::E:
      return {point.first + 1, point.second};
    case Direction::S:
      return {point.first, point.second - 1};
    case Direction::W:
      return {point.first - 1, point.second};
    default:
      return point;
  }
}

void loop() {
  debug("loop() called: ");

  if (error != 0) {
    debug(error);
    return;
  }
  
  if (state == State::FINISHED) {
    debug("Robot has finished!");
    return;
  }

  if (state == State::FOUND_PATH) {
    debug("Found shortest path, traversing points...");
    print_graph();
    for (const auto& point : s_path) {
      drive_to(point);
    }
    debug("Finished traversing points! (waiting 5s)");
    if (tcs.Read() == Color::LIGHT_RED) {
      debug("Found the finish after traversing!");
      state = State::FINISHED;
    } else {
      debug("Finish was absent at the end of the shortest path.");
      error = 1;
    }
    return;
  }

  drive:
  debug("Attempting to drive to the next cell...");
  bool can_drive_fwd = drive_fwd_one();
  if (!can_drive_fwd) {
    rot_fwd_empty();
    goto drive;
  }

  debug("We drove to another cell.");
  
  debug("Measuring colors...");
  switch (tcs.Read()) {
    case Color::LIGHT_BLUE: {
      if (state != State::FOUND_END) {
        debug("We have not found the end yet, error 2.");
        error = 2;
      } else {
        debug("Executing Dijkstra.");
        dijkstra__pws();
        debug("Found the shortest path, setting state to FOUND_PATH");
        state = State::FOUND_PATH;
      }
      debug("Found the start!");
      break;
    }
    case Color::LIGHT_RED: {
      if (state != State::EXPLORING) {
        debug("We should be back at the start by now! (error 3)");
        error = 3;
      } else {
        debug("We found the end, assigning finish to current point.");
        state = State::FOUND_END;
        finish = cp;
      }
      debug("Found the end!");
      break;
    }
    default:
      break;
  }
  
} // loop();

void dijkstra__pws() {
  const auto num_vertices = boost::num_vertices(graph);
  std::vector<vertex_descriptor> paths(num_vertices);
  std::vector<int> dist;
  dist.resize(num_vertices);
  
  auto pm = predecessor_map(boost::make_iterator_property_map(paths.begin(), get(boost::vertex_index, graph)));
  const auto dm = pm.distance_map(boost::make_iterator_property_map(dist.begin(), get(boost::vertex_index, graph)));
  vertex_descriptor start_vertex = boost::vertex(flatten_point(start), graph);
  boost::dijkstra_shortest_paths_no_color_map(graph, start_vertex, pm);

  unsigned int index = flatten_point(finish);
  unsigned int start_index = flatten_point(finish);
  Point p = graph[index];
  s_path.push_back(p);
  while (index != start_index) {
    index = paths[index];
    index--;
    Point current = graph[index];
    s_path.push_back(current);
  }

  s_path.push_back(graph[start_index]);
  std::reverse(s_path.begin(), s_path.end());
}

// Drive one cell forward
bool drive_fwd_one() {
  if (fds.Pulse() != INVALID_DISTANCE) {
    return false; // If the distance to a wall can be measured, there is no cell ahead.
  }

  /**
   * Drive forward until a major difference is found in either the right or left distance sensor, or until a timeout is exceeded (indicating a straight path to the next cell).
   * 
   * Scenario 1: When the robot (R) reaches the next cell (N), a significant distance difference is observed comming from the previous cell.
   * +---+---+---+
   * | R   N     |
   * +---+   +---+
   * |    !!!    |
   * +---+---+---+
   * 
   * Scenario 2: The robot does not find a significant distance difference so it stops after the timeout.
   * +---+---+---+
   * | R   N     |
   * +---+---+---+
   */
  debug("Driving forward.");
  const int fun_start_t = millis();
  driver.DriveForward(); // Start driving
  int margin = 3;
  do {
    static int dist_l = false;
    static int dist_r = false;
    
    int pulse_l = ceil( lds.Pulse() );
    int pulse_r = ceil( rds.Pulse() );
    
    if (dist_l & dist_r) {
      bool l_check = pulse_l + margin >= dist_l && dist_l >= pulse_l - margin;
      bool r_check = pulse_r + margin >= dist_r && dist_r >= pulse_r - margin;
      if (!l_check || !r_check) {
        debug("Found a exit adjacent to the robot!");
        break;
      }
    }
    
    dist_l = pulse_l;
    dist_r = pulse_r;
  } while (millis() < fun_start_t + 3000);
  
  delay(300); // Make sure the robot stops in the center of the cell.
  driver.Stop();

  auto next = incr_point(cp, cd);
  activate_edge(cp, next);
  cp = next;
  debug("Finished driving forward one cell.");
  return true;
} // drive_forward_one_cell();

void print_point(const Point& point, bool parentheses, bool newline) {
  char data[13];
  sprintf(data, parentheses ? "(%d, %d)" : "%d, %d", point.first, point.second);
  if (newline) {
    Serial.println(data);
  } else {
    Serial.print(data);
  }
}

void print_graph() {
  Serial.println("graph maze {");
  Serial.println("\tgraph [splines=true]");
  Serial.println();
  Serial.println("\tnode [shape=circle, fixedsize=true] {");

  auto vertices = boost::vertices(graph);
  for (auto vd = vertices.first; vd != vertices.second; vd++) {
    if (boost::out_degree(*vd, graph) == 0) {
      continue;
    }
    
    const Point vertex = graph[*vd];
    Serial.print("\t\t\"");
    print_point(vertex, true, false);
    Serial.print("\" [pos=\"");
    print_point(vertex, false, false);
    Serial.print("!\", label=\"");
    
    if (vertex == start) {
      Serial.print('S');
    } else if (vertex == finish) {
      Serial.print('F');
    } else {
      char label = flatten_point(vertex) + 'A';
      Serial.print(label);
    }
    
    Serial.println("\"]");
  }
  Serial.println("\t}");
  
  auto edges = boost::edges(graph);
  for (auto ed = edges.first; ed != edges.second; ed++) {
    const Point source = graph[boost::source(*ed, graph)];
    const Point target = graph[boost::target(*ed, graph)];
    Serial.print("\t\"");
    print_point(source, true, false);
    Serial.print("\" -- \"");
    print_point(target, true, false);
    Serial.print("\" [label=");
    auto weight = get(boost::edge_weight, graph)[*ed];
    Serial.print(weight);
    if (is_edge_in_shortest_path(source, target)) {
      Serial.print(", color=red, style=bold");
    }
    Serial.println(']');
  }
  Serial.println('}');
}
