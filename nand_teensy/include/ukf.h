#pragma once

#include <ArduinoEigenDense.h>

#define STATE_SPACE_DIM 3
typedef Eigen::Matrix<double, STATE_SPACE_DIM, 1> state_vector_t;
typedef Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state_cov_matrix_t;

#define MEASUREMENT_SPACE_DIM 2
typedef Eigen::Matrix<double, MEASUREMENT_SPACE_DIM, 1> measurement_vector_t;
typedef Eigen::Matrix<double, MEASUREMENT_SPACE_DIM, MEASUREMENT_SPACE_DIM> measurement_cov_matrix_t;

#define INPUT_SPACE_DIM 1
typedef Eigen::Matrix<double, INPUT_SPACE_DIM, 1> input_vector_t;

#define EIGEN_MAX_ITERS 50;


class UKF
{
private:
  double wheelbase;
  double speed;
  double zeroth_sigma_point_weight;

  state_cov_matrix_t process_noise;
  measurement_cov_matrix_t gps_noise;

  state_vector_t dynamics(state_vector_t state, input_vector_t input);
  state_vector_t rk4(state_vector_t state, input_vector_t input, double dt);

  void generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], double weights[2 * STATE_SPACE_DIM + 1]);

  measurement_vector_t state_to_measurement(state_vector_t vector);


public:
  UKF(double wheelbase, double zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t gps_noise);

  void set_speed(double speed);
  void set_gps_noise(double accuracy);

  void predict(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, input_vector_t input, double dt,
               state_vector_t &predicted_state_est, state_cov_matrix_t &predicted_state_cov);

  void update(state_vector_t curr_state_est, state_cov_matrix_t curr_state_cov, measurement_vector_t measurement,
              state_vector_t &updated_state_est, state_cov_matrix_t &updated_state_cov);
};

state_vector_t get_col(state_cov_matrix_t A, int i);
state_cov_matrix_t square_root(state_cov_matrix_t matrix);

class FilterState {
public:
  FilterState() :
    filter(
      // ???
      1,
      // ???,
      1/3, 
      // Process noise,
      state_cov_matrix_t {
        { 0.0001,    0.0,    0.0 },
        { 0.0,    0.0001,    0.0 },
        { 0.0,       0.0, 0.0001 }
      },
      // GPS noise,
      measurement_cov_matrix_t {
        { 0.01, 0.0  },
        { 0.0,  0.01 }
      }
    ),
    last_predict_timestamp(millis() / 1000.0)
  {}

  // X and Y are meters in UTM
  // Accuracy is in mm
  void handle_gps(double x, double y, double acc);

  // Speed measured in m/s
  void handle_encoder(double speed);

  // 
  void handle_steering(double steering);

private:
  // ukf value setup
  state_vector_t curr_state_est{{0, 0, 0}}; 

  state_cov_matrix_t curr_state_cov{{1, 0, 0},
                                    {0, 1, 0},
                                    {0, 0, 1}};

  state_vector_t predicted_state_est;
  state_cov_matrix_t predicted_state_cov;
  state_vector_t updated_state_est;
  state_cov_matrix_t updated_state_cov;

  input_vector_t current_steering { 0.0 /*stream.next_steering_angle*/ };

  UKF filter;

  // Measured in seconds
  double last_predict_timestamp;
};