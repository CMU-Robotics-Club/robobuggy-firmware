#pragma once
#include "host_comms.h" // needed for Point and PathMsg
#include <Arduino.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#define DEBUG_PRINT(x) Serial.println(x)

using host_comms::PathMsg;
using host_comms::Point;
using host_comms::StanleyDebug;
using std::uint64_t;

// Buggy position
struct Pose {
  // UTM coordinates
  double x;       // Easting
  double y;       // Northing
  double v;       // m/s
  double heading; // Heading angle of the buggy in radians.
                  // 0 = tangent line is facing east.
};

/**
 * @brief Trajectory class handles path buffer for Stanley
 */
class Trajectory {
public:
  /**
   * @brief Gets the start index in the current path
   */
  uint64_t get_first_index();

  /**
   * @brief Gets the number of points currently in the path buffer
   */
  size_t get_num_points();

  /**
   * @brief Compute the distance to all indexes in the range
   * and get the closest one
   */
  std::optional<double> get_closest_index_on_path(double front_x,
                                                  double front_y,
                                                  uint64_t start_idx,
                                                  uint64_t end_idx);

  /**
   * @brief Return the idxth point in the path if it is in range, otherwise
   * nullptr
   */
  const Point *get_by_index(uint64_t idx);

  /**
   * @brief Write (or overwrite) a path segment in the buffer
   * Returns the number of points written,
   * which is <= the number of points in the PathMsg
   */
  size_t add_segment(const PathMsg &p);

  /**
   * @brief Interpolate point at fractional index (currently LERP)
   */
  std::optional<Point> interpolate(double index);

private:
  struct Slot {
    host_comms::Point p;
    uint64_t idx;
    bool valid;
  };
  uint64_t start_idx = 0; // inclusive
  uint64_t end_idx = 0;   // exclusive
  static constexpr size_t capacity = 128;
  static constexpr int REFINE_ITERS = 16;
  std::array<Slot, capacity> buffer;
};

/**
 * @brief Stanley controller is the steering algorithm for the buggy
  Reference is this paper
  https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf

  If the stupid link doesn't work look here:
  https://github.com/CMU-Robotics-Club/robobuggy-software/wiki/Controller
*/
class StanleyController {
public:
  // From software controller_superclass.py
  // TODO: Check if this value isn't bullshit
  static constexpr double SC_WHEELBASE = 1.104; // Units meters
  static constexpr double NAND_WHEELBASE = 1.3; // Units meters

  explicit StanleyController(double wheelbase, double start_index,
                             bool use_heading_rate_error)
      : wheelbase(wheelbase), use_heading_rate_error(use_heading_rate_error),
        current_traj_index(start_index) {}

  // Returns the steering angle in degrees or std::nullopt if we've run out of
  // path (if this happens, we should just kill ourselves)
  std::optional<double> compute_control(const Pose &current_pose,
                                        Trajectory &path);

  // Send this as-is with host_comms::sc_send_stanley_debug(stanley.debug())
  const StanleyDebug &debug() const { return dbg; }

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
  StanleyDebug dbg{};
};