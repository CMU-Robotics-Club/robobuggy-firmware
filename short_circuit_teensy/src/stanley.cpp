#include <Arduino.h>
#include "host_comms.h" // needed for Point and PathMsg

#include "stanley.h"

// hello

// Step 2: write Trajectory class
Trajectory::Trajectory()
{
  read_index = 0;
  write_index = 0;
}

int Trajectory::get_num_points()
{
  return (write_index - read_index) % BUFFER_SIZE;
}

int Trajectory::get_closest_index_on_path(
  double front_index,
  double left_index,
  int start_index,
  int end_index
)
{

}

Point Trajectory::get_by_index(int index)
{
  return buffer[index % BUFFER_SIZE];
}

bool Trajectory::add_point(Point p) 
{
  int next_write_index = (write_index + 1) % BUFFER_SIZE;
  
  if(next_write_index == read_index){
    return false; // we're full
  }
  // write p to the buffer at write_index
  buffer[write_index] = p;  
  
  write_index = next_write_index;
  return true;

};

bool add_segment(PathMsg p) 
{
  for (int i = 0; i < num_pts; i++){
    add_point(pts[i]);
  }
  /*
  struct PathMsg {
  uint64_t num_pts;
  uint64_t start_idx; // index of pts[0]
  float64 dist; // arc length distance between points
  Point[] pts;  // VLA, size = num_pts
  };
  */

  // what if the PathMsg start_idx is not the next index after the last written point?
    // how do I know how far back to change the write_index?
    // we must keep track of the last end point index
    // subtract last point index from new start write index
    // e.g if last point index is 99 and new point index is 100,
    // we should not move the write_index at all
    // offset = (new point index - (last point index + 1))
    // if the offset is negative, we should add (subtract the magnitude) of the offset
    // from the write_index
    // otherwise, if it is positive, there is a gap of trajectory points,
    // some error should be notified. 
}

// Step 3: write Stanley class