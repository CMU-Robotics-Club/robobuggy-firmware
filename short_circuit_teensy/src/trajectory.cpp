#include <Arduino.h>

#include "host_comms.h" // needed for Point and PathMsg
#include <array>
#include <cstdint>
#include <stdint.h>

#include "stanley.h"

uint64_t Trajectory::get_first_index() { return start_idx; }

size_t Trajectory::get_num_points() { return end_idx - start_idx; }

std::optional<double> Trajectory::get_closest_index_on_path(double fx,
                                                            double fy,
                                                            uint64_t lo,
                                                            uint64_t hi) {

  if (end_idx == start_idx)
    return std::nullopt;
  lo = max(lo, start_idx);
  hi = min(hi, end_idx - 1);
  if (lo > hi) {
    DEBUG_PRINT("[TRAJECTORY get_closest_index_on_path] ERROR: specified start "
                "index > end index");
    return std::nullopt;
  }

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
  if (!best_vertex) {
    DEBUG_PRINT("[TRAJECTORY get_closest_index_on_path] couldn't find a best vertex");
    return std::nullopt;
  }
  const uint64_t b = *best_vertex;

  // Fine: interpolate between b-1 and b+1 for better result
  auto dist2 = [&](double idx) {
    const auto p = interpolate(idx);
    if (!p)
      return (double)INFINITY;
    const double dx = p->x - fx, dy = p->y - fy;
    return dx * dx + dy * dy;
  };

  // Close the search window
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
  if (idx < start_idx || idx >= end_idx)
    return nullptr;
  const Slot &s = buffer[idx % capacity];
  if (!s.valid || s.idx != idx)
    return nullptr;
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
  
  // write/overwrite points
  size_t num_points_written = 0;
  for (uint64_t i = p.start_idx; i < segment_end; i++) {
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

std::optional<PathSample> Trajectory::interpolate(double index) {
  if (end_idx == start_idx)
    return std::nullopt;
  index = constrain(index, (double)start_idx, (double)(end_idx - 1));

  uint64_t i0 = (uint64_t)std::floor(index);
  double t = index - (double)i0;
  // On the last point there is no segment after it, use the one before, at t = 1
  if (t == 0.0 && i0 > start_idx && !get_by_index(i0 + 1)) {
    i0 -= 1;
    t = 1.0;
  }

  const Point *p0 = get_by_index(i0);
  const Point *p1 = get_by_index(i0 + 1);
  if (!p0 || !p1)
    return std::nullopt; // gap, or a single point

  // Relative to p0
  const double dx = p1->x - p0->x;
  const double dy = p1->y - p0->y;
  const double chord = std::hypot(dx, dy);
  auto tangent = [&](const Point &p, double &mx, double &my) {
    const double n = std::hypot(p.vx, p.vy);
    if (n > 0.0) {
      mx = p.vx / n * chord;
      my = p.vy / n * chord;
    } else {
      mx = dx;
      my = dy;
    }
  };
  double m0x, m0y, m1x, m1y;
  tangent(*p0, m0x, m0y);
  tangent(*p1, m1x, m1y);

  const double t2 = t * t, t3 = t2 * t;
  const double h10 = t3 - 2 * t2 + t, h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
  const double d10 = 3 * t2 - 4 * t + 1, d01 = -6 * t2 + 6 * t, d11 = 3 * t2 - 2 * t;
  const double dd10 = 6 * t - 4, dd01 = -12 * t + 6, dd11 = 6 * t - 2;

  const double xp = d10 * m0x + d01 * dx + d11 * m1x;
  const double yp = d10 * m0y + d01 * dy + d11 * m1y;
  const double xpp = dd10 * m0x + dd01 * dx + dd11 * m1x;
  const double ypp = dd10 * m0y + dd01 * dy + dd11 * m1y;
  const double speed2 = xp * xp + yp * yp;

  PathSample s;
  s.x = p0->x + h10 * m0x + h01 * dx + h11 * m1x;
  s.y = p0->y + h10 * m0y + h01 * dy + h11 * m1y;
  if (speed2 > 0.0) {
    s.heading = std::atan2(yp, xp);
    s.curvature = (xp * ypp - yp * xpp) / (speed2 * std::sqrt(speed2));
  } else {
    s.heading = std::atan2(p0->vy, p0->vx);
    s.curvature = 0.0;
  }
  return s;
}