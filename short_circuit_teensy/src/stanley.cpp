#include "host_comms.h" // needed for Point and PathMsg
#include <Arduino.h>
#include <cmath>

#include "stanley.h"

namespace {
// Canonical to doing (e + pi) % (2pi) - pi in python
inline double wrap_angle(double a) { return std::remainder(a, 2.0 * M_PI); }

/**
 * @brief Returns reference point at a fractional index, linearly interpolated
 * between the two neighbouring points
 */
Point interpolate(Trajectory &path, double index) {
  const int last = path.get_num_points() - 1;
  int i0 = static_cast<int>(std::floor(index));
  if (i0 > last - 1)
    i0 = last - 1;
  const double a = constrain(index - i0, 0.0, 1.0);

  const Point *p0_ptr = path.get_by_index(i0);
  const Point *p1_ptr = path.get_by_index(i0 + 1);
  if (!(p0_ptr && p1_ptr)) {
    DEBUG_PRINT(
        "[STANLEY interpolate] ERROR: requested points not on the path");
    return {};
  }
  const Point p0 = *p0_ptr;
  const Point p1 = *p1_ptr;
  return {
      p0.x + a * (p1.x - p0.x),
      p0.y + a * (p1.y - p0.y),
      wrap_angle(p0.heading_tangent +
                 a * wrap_angle(p1.heading_tangent - p0.heading_tangent)),
      p0.curvature + a * (p1.curvature - p0.curvature),
  };
}
} // namespace

StanleyController::StanleyController(double wheelbase, double start_index,
                                     bool use_heading_rate_error)
    : wheelbase(wheelbase), use_heading_rate_error(use_heading_rate_error),
      current_traj_index(start_index) {}

std::optional<double>
StanleyController::compute_control(const Pose &current_pose, Trajectory &path) {
  dbg = {};

  const int first = path.get_first_index();
  const int last = first + path.get_num_points() - 1;

  // Out of path, you should shoot Rishi if this happens
  if (current_traj_index >= last || last - first < 1) {
    return std::nullopt;
  }
  // Path buffer rolled past us, avoid using no
  if (current_traj_index < first) {
    current_traj_index = first;
  }

  const double speed = current_pose.v;
  const double heading = current_pose.heading;
  const double yaw_rate = 0;       // current_pose.heading_rate;
  const double x = current_pose.x; // Easting
  const double y = current_pose.y; // Northing

  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);

  // Front axle position
  const double front_x = x + wheelbase * cos_h;
  const double front_y = y + wheelbase * sin_h;

  // Coarse search, finding nearest buffered point near where we were
  const int cur = static_cast<int>(std::floor(current_traj_index));
  const size_t nearest = path.get_closest_index_on_path(
      front_x, front_y, cur - SEARCH_BEHIND, cur + SEARCH_AHEAD);

  // Fine search, project the front axle onto the tangent at the nearest point
  double traj_index = nearest;
  const double spacing = path.get_spacing();
  if (spacing > 0.0) {
    const Point *n_ptr = path.get_by_index(nearest);
    if (!n_ptr) {
      DEBUG_PRINT(
          "[STANLEY compute_control] ERROR: requested a point not on the path");
      return 0;
    }
    const Point n = *n_ptr;
    const double along = (front_x - n.x) * std::cos(n.heading_tangent) +
                         (front_y - n.y) * std::sin(n.heading_tangent);
    traj_index = constrain(nearest + along / spacing, nearest - 1.0, nearest + 1.0);
    traj_index = constrain(traj_index, first, last);
  }

  // Don't go backwards
  if (traj_index > current_traj_index) {
    current_traj_index = traj_index;
  }

  const Point ref = interpolate(path, current_traj_index);

  // Heading error, using the path heading at the reference index
  const double error_heading = wrap_angle(ref.heading_tangent - heading);

  // Signed cross-track error, distance from the front axle to the path's
  // tangent line at ref.
  const double tan_x = std::cos(ref.heading_tangent);
  const double tan_y = std::sin(ref.heading_tangent);
  const double error_dist =
      (front_x - ref.x) * tan_y - (front_y - ref.y) * tan_x;

  const double cross_track_component = std::atan2(
      (CROSS_TRACK_GAIN + CROSS_TRACK_GAIN_SPEED * speed) * error_dist,
      speed + K_SOFT);

  // Expected yaw rate from path curvature, r_traj = curvature * speed (rad/s)
  const double r_traj = ref.curvature * speed;
  const double yaw_rate_error = r_traj - yaw_rate;
  const double yaw_correction = K_D_YAW * yaw_rate_error;

  // Steering command
  double steering_cmd = error_heading + cross_track_component;
  if (use_heading_rate_error) {
    steering_cmd += yaw_correction;
  }
  steering_cmd = constrain(steering_cmd, -MAX_STEER, MAX_STEER); // radians

  // error_pose: ref point in the buggy frame (x along heading, y = lateral),
  // then shifted back to the rear wheel
  const double dx = ref.x - x;
  const double dy = ref.y - y;
  const double local_x = dx * cos_h + dy * sin_h;
  const double local_y = -dx * sin_h + dy * cos_h;

  // Fill debug packet
  dbg.error_pose_x = local_x - wheelbase;
  dbg.error_pose_y = local_y;
  dbg.reference_navsat_easting = ref.x;
  dbg.reference_navsat_northing = ref.y;
  dbg.error_dist = error_dist;
  dbg.error_heading = error_heading;
  dbg.yaw_rate_error = yaw_rate_error;

  // convert to degrees
  return steering_cmd * (180.0 / M_PI);
}