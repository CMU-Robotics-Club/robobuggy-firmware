#include <Arduino.h>

#include "host_comms.h" // needed for Point and PathMsg
#include <array>
#include <cstdint>
#include <stdint.h>

#include "stanley.h"

uint64_t Trajectory::get_first_index() { return start_idx; }

size_t Trajectory::get_num_points() { return (end_idx - start_idx) % capacity; }

double Trajectory::get_spacing() { return spacing; }

uint64_t Trajectory::get_closest_index_on_path(double front_x, double front_y,
                                               uint64_t start_index,
                                               uint64_t end_index) {

  if (!(start_index <= end_index)) {
    DEBUG_PRINT("[TRAJECTORY get_closest_index_on_path] ERROR: specified start "
                "index > end index");
    return 0;
  }

  // loop thru the buffer from start to end
  // find the point with the min distance
  // there's probably a nicer way to do this but i have written
  // it explicitly
  size_t min_idx;
  double min_dist = SIZE_MAX;
  for (size_t i = start_index; i <= end_index; i++) {
    // find the distance
    Slot s = buffer[i % capacity];
    Point p = s.p;
    if (!(i == s.idx && s.valid)) // skip invalid points
      continue;
    double distance = sqrt(pow(p.x - front_x, 2) + pow(p.y - front_y, 2));
    if (distance < min_dist) {
      min_dist = distance;
      min_idx = i;
    }
  }

  // interpolate between points min_idx-1 and +1 for higher accuracy
  return min_idx;
}

const Point *Trajectory::get_by_index(uint64_t idx) {
  Slot s = buffer[idx % capacity];
  Point p = s.p;
  if (!(start_idx <= idx && idx < end_idx)) {
    DEBUG_PRINT("[TRAJECTORY get_by_index]: ERROR: index not in range");
    return nullptr;
  }
  if (!(idx == s.idx && s.valid)) {
    DEBUG_PRINT("[TRAJECTORY get_by_index]: ERROR: index in gap");
    return nullptr;
  }
  return &p;
}

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
size_t Trajectory::add_segment(const PathMsg &p) {
  if (p.num_pts == 0)
    return 0;

  if (p.start_idx < start_idx) {
    DEBUG_PRINT("[TRAJECTORY add_segment] ERROR: Trying to write segment where "
                "start is before "
                "current path start");
    return 0;
  }

  uint64_t segment_end = p.start_idx + p.num_pts; // exclusive

  // set spacing
  spacing = p.dist;

  // invalidate any gap
  for (size_t i = end_idx % capacity; i < p.start_idx % capacity; i++) {
    Slot *s = &buffer[i];
    s->valid = false;
  }

  // write/overwrite points
  for (size_t i = p.start_idx; i < segment_end; i++) {
    Slot *s = &buffer[i % capacity];
    s->p = p.pts[i - p.start_idx];
    s->valid = true;
    s->idx = i;
  }

  // update start and end indices
  end_idx = max(end_idx, segment_end) % capacity; // exclusive
  start_idx = max(start_idx, end_idx - capacity) % capacity; // inclusive
}