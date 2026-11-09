#pragma once

#include "host_comms.h" // needed for Point and PathMsg
#include <Arduino.h>
#include <optional>
#include <cmath>

#define BUFFER_SIZE 100

struct Point {
  double x; // UTM coordinates
  double y;
  double heading_tangent; // radians, 0 = tangent is facing east
  double curvature;
};

// SW -> FW
// using this for stanley
struct PathMsg {
  uint64_t num_pts;
  uint64_t start_idx; // index of pts[0]
  double dist;        // arc length distance between points
  Point pts[];        // VLA, size = num_pts
};

// Step 1: write pose struct
struct Pose {
  // UTM coordinates
  double x; // Our Easting
  double y; // Our Northing
  double v;
  double heading; // Heading angle of the buggy in radians.
                  // 0 = tangent line is facing east.
};

class Trajectory {
public:
  Trajectory();
  int get_num_points();
  int get_closest_index_on_path(double front_x, double front_y, int start_index,
                                int end_index);
  Point &get_by_index(int index);
  bool add_segment(PathMsg &p); // write to buffer
private:
  int read_index;
  int write_index;
  Point &buffer[BUFFER_SIZE];
  bool add_point(Point &p); // write to buffer
};

// Step 3: Stanley controller (front axle used as reference point)
// https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf
class StanleyController {
public:
  // From software controller_superclass.py
  // TODO: Check if this value isn't bullshit
  static constexpr double SC_WHEELBASE = 1.104; // Units meters
  static constexpr double NAND_WHEELBASE = 1.3; // Units meters

  explicit StanleyController(double wheelbase = SC_WHEELBASE,
                             double start_index = 0,
                             bool use_heading_rate_error = false);

  // Returns the steering angle in degrees or std::nullopt if we've run out of
  // path (if this happens, we should just kill ourselves)
  std::optional<double> compute_control(const Pose &current_pose,
                                        const Trajectory &path);

  // Send this as-is with host_comms::sc_send_stanley_debug(stanley.debug())
  const host_comms::StanleyDebug &debug() const { return dbg; }

private:
  static constexpr double CROSS_TRACK_GAIN = 1.3;
  static constexpr double CROSS_TRACK_GAIN_SPEED = 0.08;
  static constexpr double K_SOFT = 1.0;           // m/s
  static constexpr double K_D_YAW = 0.012;        // rad / (rad/s)
  static constexpr double MAX_STEER = M_PI / 9.0; // 20 deg

  // search window around current index
  static constexpr int SEARCH_BEHIND = 20;
  static constexpr int SEARCH_AHEAD = 50;

  const double wheelbase;
  const bool use_heading_rate_error;
  double current_traj_index; // fractional absolute index
  host_comms::StanleyDebug dbg{};
};