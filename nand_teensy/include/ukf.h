#pragma once

#include <ArduinoEigenDense.h>

/**
 * @brief STATE SPACE
 * Vector space of dimension 3.
 * First parameter represents x-location.
 * Second parameter represents y-location.
 * Third parameter represents speed.
 */
#define STATE_SPACE_DIM 3
typedef Eigen::Matrix<double, STATE_SPACE_DIM, 1> state_vector_t;
typedef Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state_cov_matrix_t;

/**
 * @brief MEASUREMENT SPACE.
 * Vector space of dimension 2.
 * First parameter represents x-location.
 * Second parameter represents y-location.
 */
#define MEASUREMENT_SPACE_DIM 2
typedef Eigen::Matrix<double, MEASUREMENT_SPACE_DIM, 1> measurement_vector_t;
typedef Eigen::Matrix<double, MEASUREMENT_SPACE_DIM, MEASUREMENT_SPACE_DIM> measurement_cov_matrix_t;

#define INPUT_SPACE_DIM 1
typedef Eigen::Matrix<double, INPUT_SPACE_DIM, 1> input_vector_t;

#define EIGEN_MAX_ITERS 50

#define MOVING_THRESHOLD 0

class UKF
{
private:
  double wheelbase;
  double zeroth_sigma_point_weight;

  state_cov_matrix_t process_noise;
  measurement_cov_matrix_t gps_noise;

  state_vector_t dynamics(state_vector_t state, input_vector_t input);
  state_vector_t rk4(state_vector_t state, input_vector_t input, double dt);

  void generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], double weights[2 * STATE_SPACE_DIM + 1]);

  measurement_vector_t state_to_measurement(state_vector_t vector);

public:
  UKF(double wheelbase, double zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t gps_noise);

  double speed;
  void set_speed(double speed);
  /**
   * @brief Sets the UKF's internal gps measurement covariance matrix.
   * 
   * Some math is involved to go from accuracy provided to covariance matrix.
   * 
   * @param accuracy Horizontal accuracy reading from the ZED-F9P, in millimeters.
   */
  void set_gps_noise(double accuracy);

  /**
   * @brief TODO WRITE DESCRIPTOIN
   * 
   * @param input 
   * @param dt   seconds since last time predict was called
   */
  void predict(input_vector_t input, double dt);

  void update(measurement_vector_t measurement);
  
  /**
   * Initial estimations for state and covariance.
   * 
   */
  state_vector_t curr_state_est{{0, 0, PI}}; 
  state_cov_matrix_t curr_state_cov{{1, 0, 0},
                                    {0, 1, 0},
                                    {0, 0, 1}};
};

state_vector_t get_col(state_cov_matrix_t A, int i);
state_cov_matrix_t square_root(state_cov_matrix_t matrix);