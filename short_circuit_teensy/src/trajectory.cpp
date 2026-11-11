#include <Arduino.h>

#include "host_comms.h" // needed for Point and PathMsg
#include <array>
#include <cstdint>
#include <stdint.h>

#include "stanley.h"

uint64_t Trajectory::get_first_index() { return start_idx; }

size_t Trajectory::get_num_points() { return end_idx - start_idx; }

double Trajectory::get_spacing() { return spacing; }

std::optional<double> Trajectory::get_closest_index_on_path(double fx,
                                                            double fy,
                                                            uint64_t lo,
                                                            uint64_t hi) {

  if (!(lo <= hi)) {
    DEBUG_PRINT("[TRAJECTORY get_closest_index_on_path] ERROR: specified start "
                "index > end index");
    return 0;
  }

  if (end_idx == start_idx)
    return std::nullopt;
  lo = max(lo, start_idx);
  hi = min(hi, end_idx - 1);

  // Coarse: find min distance
  double best_d2 = INFINITY;
  std::optional<uint64_t> best_vertex;
  for (uint64_t i = lo; i <= hi; i++) {
    const Point *p = get_by_index(i);
    if (!p)
      continue;
    const double dx = p->x - fx, dy = p->y - fy;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_vertex = i;
    }
  }
  if (!best_vertex)
    return std::nullopt;
  const uint64_t b = *best_vertex;

  // Fine: interpolate between b-1 and b+1 for better result
  auto dist2 = [&](double idx) {
    const auto p = interpolate(idx);
    if (!p)
      return (double)INFINITY;
    const double dx = p->x - fx, dy = p->y - fy;
    return dx * dx + dy * dy;
  };

  double a = (b > 0 && get_by_index(b - 1)) ? (double)(b - 1) : (double)b;
  double c = get_by_index(b + 1) ? (double)(b + 1) : (double)b;
  for (int it = 0; it < REFINE_ITERS; it++) {
    const double m1 = a + (c - a) / 3.0;
    const double m2 = c - (c - a) / 3.0;
    if (dist2(m1) < dist2(m2))
      c = m2;
    else
      a = m1;
  }
  return 0.5 * (a + c);
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
  return &s.p;
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

  if (end_idx == start_idx) { // empty buffer means reset to beginning
    start_idx = end_idx = p.start_idx;
  }

  if (p.start_idx < start_idx) {
    DEBUG_PRINT("[TRAJECTORY add_segment] ERROR: Trying to write segment where "
                "start is before "
                "current path start");
    return 0;
  }

  uint64_t segment_end = p.start_idx + p.num_pts; // exclusive

  // set spacing
  spacing = p.dist;

  // write/overwrite points
  size_t num_points_written = 0;
  for (size_t i = p.start_idx; i < segment_end; i++) {
    Slot &s = buffer[i % capacity];
    s.p = p.pts[i - p.start_idx];
    s.valid = true;
    s.idx = i;
    num_points_written++;
  }

  // update start and end indices
  end_idx = max(end_idx, segment_end); // exclusive
  if (end_idx > capacity)
    start_idx = max(start_idx, end_idx - capacity); // inclusive

  return num_points_written;
}

// currently lerp
std::optional<Point> Trajectory::interpolate(double index) {
  if (end_idx == start_idx)
    return std::nullopt;
  index = constrain(index, (double)start_idx, (double)(end_idx - 1));

  const uint64_t i0 = (uint64_t)std::floor(index);
  const double a = index - (double)i0;

  const Point *p0 = get_by_index(i0);
  if (a == 0.0) { // exactly on a vertex: no neighbor needed
    if (!p0)
      return std::nullopt;
    return *p0;
  }
  const Point *p1 = get_by_index(i0 + 1);
  if (!p0 || !p1)
    return std::nullopt; // gap

  Point pf;
  pf.x = p0->x + a * (p1->x - p0->x);
  pf.y = p0->y + a * (p1->y - p0->y);
  pf.heading_tangent = remainder(
      p0->heading_tangent +
          a * remainder(p1->heading_tangent - p0->heading_tangent, 2.0 * M_PI),
      2.0 * M_PI);
  pf.curvature = p0->curvature + a * (p1->curvature - p0->curvature);
  return pf;
}