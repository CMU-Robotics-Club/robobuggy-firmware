#include "host_comms.h" // needed for Point and PathMsg
#include <Arduino.h>
#include <array>

#include "stanley.h"

#define BUF_SIZE 100

struct Point {
  double x; // UTM coordinates
  double y;
  double heading_tangent; // radians, 0 = tangent is facing east
  double curvature;
};

class Trajectory {

public:
  /**
   * Constructor initializes all private variables
   */
  Trajectory() {
    current_idx = 0;
    num_points = 0;
  }

  Trajectory(size_t Current_idx, size_t Num_points) {
    current_idx = Current_idx;
    num_points = Num_points;
  }

  size_t get_num_points() { return num_points; }

  size_t get_closest_index_on_path(
    double front_x, double front_y,
    int start_index, int end_index) {

      if (!(start_index <= end_index)) return 0;

      // loop thru the buffer from start to end
      // find the point with the min distance
      // there's probably a nicer way to do this but i have written
      // it explicitly
      size_t min_idx;
      double min_dist = 999;
      for (size_t i = start_index; i <= end_index; i++) {
        // find the distance
        Point p = buffer[i % BUF_SIZE];
        double distance = sqrt(pow(p.x - front_x, 2) + pow(p.y - front_y, 2));
        // replace min distance
        if (distance < min_dist) {
          min_dist = distance;
          min_idx = i;
        }
      }

      // interpolate for higher accuracy here

      return min_idx;
  }

  Point &get_by_index(size_t idx) { return buffer[idx % BUF_SIZE]; }

  /**
   * struct PathMsg {
      uint64_t num_pts;
      uint64_t start_idx; // index of pts[0]
      float64 dist; // arc length distance between points
      Point[] pts;  // VLA, size = num_pts
    };

    Make a for loop
    change the start index in the buffer to the values of the Msg, and loop that
   with increasing index till you reach the Msg size. Also loop back to the
   beginning of the buffer when you reach the end with %
  */
  size_t add_segment(PathMsg &p) {

    size_t start = p.start_idx;
    size_t end = p.start_idx + p.num_pts;

    for (size_t i = start; i < end; i++) {
      buffer[i % BUF_SIZE] = p.pts[i - start];
      if (num_points < 100)
        num_points++;
    }
  }

private:
  std::array<Point, BUF_SIZE> buffer;
  size_t current_idx;
  size_t num_points;
};