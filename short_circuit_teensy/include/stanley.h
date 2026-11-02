#pragma once

#include <Arduino.h>
#include "host_comms.h" // needed for Point and PathMsg

#define BUFFER_SIZE 100

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
    int get_closest_index_on_path(
      double front_x, 
      double front_y, 
      int start_index, 
      int end_index
    );
    Point get_by_index(int index);
    bool add_segment(PathMsg p);          // write to buffer
  private:
    int read_index; 
    int write_index; 
    Point buffer[BUFFER_SIZE]; 
    bool add_point(Point p); // write to buffer
};

class StanleyController {
  public:
    StanleyController(); // constructor
    void compute_control(const pose current_pose, const Trajectory& path);
  private:

};