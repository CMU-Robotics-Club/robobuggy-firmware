#pragma once

#include <ArduinoEigenDense.h>

/**
 * @brief STATE SPACE
 * Vector space of dimension 4.
 * First state variable represents x-location.
 * Second state variable represents y-location.
 * Third state variable represents heading in radians.
 * 0 is positive along the x-axis, increasing CCW.
 */
#define STATE_SPACE_DIM 4
typedef Eigen::Matrix<double, STATE_SPACE_DIM, 1> state_vector_t;
typedef Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state_cov_matrix_t;

/**
 * Vector space of dimension 2.
 * First parameter represents x-location (easting, meters).
 * Second parameter represents y-location (northing, meters).
 */
#define GPS_MEASUREMENT_SPACE_DIM 2
typedef Eigen::Matrix<double, GPS_MEASUREMENT_SPACE_DIM, 1> measurement_vector_t;
typedef Eigen::Matrix<double, GPS_MEASUREMENT_SPACE_DIM, GPS_MEASUREMENT_SPACE_DIM> measurement_cov_matrix_t;

#define SPEED_MEASUREMENT_SPACE_DIM 1
typedef double speed_t;

#define INPUT_SPACE_DIM 1
typedef Eigen::Matrix<double, INPUT_SPACE_DIM, 1> input_vector_t;

#define EIGEN_MAX_ITERS 50

#define MOVING_THRESHOLD 0.5

class UKF
{
private:
  double wheelbase;
  double zeroth_sigma_point_weight;

  state_cov_matrix_t process_noise;
  measurement_cov_matrix_t gps_noise;
  speed_t speed_noise;

  state_vector_t dynamics(state_vector_t state, input_vector_t input);
  state_vector_t rk4(state_vector_t state, input_vector_t input, double dt);

  void generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], double weights[2 * STATE_SPACE_DIM + 1]);

  measurement_vector_t state_to_gps_measurement(state_vector_t state);
  speed_t state_to_speed_measurement(state_vector_t state);

public:
  UKF(double wheelbase, double zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t gps_noise, speed_t speed_noise);

  /**
   * @brief Sets the UKF's internal gps measurement covariance matrix.
   *
   * Some math is involved to go from "accuracy" provided (circular error probable) to covariance matrix.
   *
   * @param accuracy Horizontal accuracy reading from the ZED-F9P, in millimeters.
   */
  void set_gps_noise(double accuracy);

  /**
   * @brief Forward simulates the state evolution and corresponding growth in estimate covariance
   * given a steering input and timestep dt.
   *
   * @param input
   * @param dt   seconds since last time predict was called
   */
  void predict(input_vector_t input, double dt);

  /**
   * @brief Computes a Kalman update for a measurement of position
   *
   * @param gps_measurement Two dimensional vector of (Easting, Northing) in meters
   */
  void update_gps(measurement_vector_t gps_measurement);

  /**
   * @brief Computes a Kalman update for a measurement of speed
   *
   * @param speed m/s measured from the encoder
   */
  void update_speed(speed_t speed_measurement);

  /**
   * Initial estimations for state and covariance.
   *
   */
  state_vector_t curr_state_est{{0, 0, PI, 0}};
  state_cov_matrix_t curr_state_cov{{1e-1, 0, 0, 0},
                                    {0, 1e-1, 0, 0},
                                    {0, 0, 1e-1, 0},
                                    {0, 0, 0, 1e-1}};
};

state_vector_t get_col(state_cov_matrix_t A, int i);
state_cov_matrix_t square_root(state_cov_matrix_t matrix);