#include "host_comms.h" // needed for Point and PathMsg
#include <Arduino.h>
#include <cmath>

#include "stanley.h"

std::optional<double>
StanleyController::compute_control(const Pose &current_pose, Trajectory &path) {
  dbg = {};

  const int first = path.get_first_index();
  const int last = first + path.get_num_points() - 1;

  // Out of path, you should shoot Rishi if this happens
  if (current_traj_index >= last || last - first < 1) {
    //Serial.println("ERROR: Out of path");
    return std::nullopt;
  }
  // Path buffer rolled past us, avoid using no
  if (current_traj_index < first) {
    current_traj_index = first;
  }

  const double speed = max(current_pose.v, 0.0); // enforce positive
  const double heading = current_pose.heading;
  const double yaw_rate = 0;       // current_pose.heading_rate;
  const double x = current_pose.x; // Easting
  const double y = current_pose.y; // Northing

  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);

  // Front axle position
  const double front_x = x + wheelbase * cos_h;
  const double front_y = y + wheelbase * sin_h;
  Serial.print(x);
  Serial.print(" ");
  Serial.print(wheelbase);
  Serial.print(" ");
  Serial.println(cos_h);

  // Finding nearest buffered point near where we were
  const int cur = static_cast<int>(std::floor(current_traj_index));
  const int lo = max(first, cur - SEARCH_BEHIND);
  const int hi = min(last, cur + SEARCH_AHEAD);
  const auto nearest_opt =
      path.get_closest_index_on_path(front_x, front_y, lo, hi);
  if (!nearest_opt) {
    /*Serial.print("ERROR: couldn't find closest index. lo=");
    Serial.print(lo);
    Serial.print(" hi=");
    Serial.print(hi);
    Serial.print(" fx,fy=");
    Serial.print(front_x);
    Serial.println(front_y);*/
    return std::nullopt;
  }

  double traj_index = constrain(*nearest_opt, (double)first, (double)last);

  // Don't go backwards
  if (traj_index > current_traj_index) {
    current_traj_index = traj_index;
  }
  
  const auto ref_opt = path.interpolate(current_traj_index);
  if (!ref_opt) {
    Serial.println("Error: couldn't interpolate");
    return std::nullopt;
  }
  const Point ref = *ref_opt;

  // Heading error, using the path heading at the reference index
  const double error_heading = remainder(ref.heading_tangent - heading, TWO_PI);

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

  // convert to degrees
  steering_cmd *= RAD_TO_DEG;

  // Fill debug packet
  dbg.steering_command = steering_cmd;
  dbg.error_pose_x = local_x - wheelbase;
  dbg.error_pose_y = local_y;
  dbg.reference_navsat_easting = ref.x;
  dbg.reference_navsat_northing = ref.y;
  dbg.error_dist = error_dist;
  dbg.error_heading = error_heading;
  dbg.yaw_rate_error = yaw_rate_error;

  return steering_cmd;
}