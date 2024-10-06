#include "ukf.h"
#include <math.h>
#include <Arduino.h>

UKF::UKF(double wheelbase, double zeroth_sigma_point_weight, state_cov_matrix_t process_noise, measurement_cov_matrix_t gps_noise)
{
  this->wheelbase = wheelbase;
  this->zeroth_sigma_point_weight = zeroth_sigma_point_weight;
  this->speed = 0;
  this->process_noise = process_noise;
  this->gps_noise = gps_noise;
}

state_vector_t get_col(state_cov_matrix_t matrix, int col)
{
  state_vector_t c;
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    c(i, 0) = matrix(i, col);
  }
  return c;
}

double magnitude(state_vector_t vector)
{
  double sum = 0;
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    sum += vector(i, 0) * vector(i, 0);
  }
  return sqrt(sum);
}

/**
 * @brief Does Gram-Schmidt algorithm.
 * Computes and returns an orthogonal matrix whose columns form the orthonormal basis for the columnspace of the input matrix.
 */
state_cov_matrix_t gram_schmidt(state_cov_matrix_t matrix)
{
  state_cov_matrix_t Q;

  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    state_vector_t x = get_col(matrix, i);
    state_vector_t projection; // projection of x onto the span of the previous i-1 columns
    projection.fill(0);
    for (int j = 0; j < i - 1; j++)
    {
      state_vector_t q = get_col(Q, j);
      double s = q.transpose() * x;
      projection += q * s;
    }

    x -= projection;

    state_vector_t q = x / magnitude(x);

    // writing q into the respective column of Q
    for (int j = 0; j < STATE_SPACE_DIM; j++)
    {
      Q(j, i) = q(j, 0);
    }
  }

  return Q;
}

/**
 * @brief Computes and returns the square root of the given state covariance matrix.
 */
state_cov_matrix_t square_root(state_cov_matrix_t matrix)
{
  Eigen::EigenSolver<state_cov_matrix_t> solver(matrix);

  state_cov_matrix_t D;
  D.fill(0);
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    D(i, i) = solver.eigenvalues()[i].real();
  }

  state_cov_matrix_t eigenvectors;
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    for (int j = 0; j < STATE_SPACE_DIM; j++)
    {
      eigenvectors(i, j) = solver.eigenvectors()(i, j).real();
    }
  }
  state_cov_matrix_t Q = gram_schmidt(eigenvectors);

  // D is a diagonal matrix
  // elements along D's diagonal are eigenvalues
  // take the square root of D
  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    D(i, i) = sqrt(D(i, i));
  }

  return (Q * D) * Q.transpose();
}

/**
 * @brief Generates sigma points given a mean and covariance.
 *
 * @param mean Vector in state space
 * @param covariance Covariance matrix (in state space)
 * @param sigmas Pointer to a list of 2*STATE_SPACE_DIM + 1 state space vectors for this function to write to
 * @param weights Pointer to a list of 2*STATE_SPACE_DIM + 1 double for this function to write to.
 */
void UKF::generate_sigmas(state_vector_t mean, state_cov_matrix_t covariance, state_vector_t sigmas[2 * STATE_SPACE_DIM + 1], double weights[2 * STATE_SPACE_DIM + 1])
{
  state_cov_matrix_t A = square_root(covariance);

  weights[0] = this->zeroth_sigma_point_weight;
  sigmas[0] = mean;

  for (int i = 0; i < STATE_SPACE_DIM; i++)
  {
    double s = sqrt(STATE_SPACE_DIM / (1 - weights[0]));
    state_vector_t A_col = get_col(A, i);
    A_col = A_col * s;

    sigmas[i + 1] = mean + A_col;
    sigmas[i + 1 + STATE_SPACE_DIM] = mean - A_col;
  }

  for (int i = 1; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    weights[i] = (1 - weights[0]) / (2 * STATE_SPACE_DIM);
  }
}

/**
 * @brief Calculates and returns the dynamics of the system
 */
state_vector_t UKF::dynamics(state_vector_t state, input_vector_t input)
{
  state_vector_t x;
  x(0, 0) = this->speed * cos(state(2, 0));
  x(1, 0) = this->speed * sin(state(2, 0));
  x(2, 0) = this->speed * tan(input(0, 0)) / this->wheelbase;
  return x;
}

state_vector_t UKF::rk4(state_vector_t state, input_vector_t input, double dt)
{
  state_vector_t k1 = this->dynamics(state, input);
  state_vector_t k2 = this->dynamics(state + (k1 * (dt / 2)), input);
  state_vector_t k3 = this->dynamics(state + (k2 * (dt / 2)), input);
  state_vector_t k4 = this->dynamics(state + (k3 * dt), input);

  return state + ((k1 + (k2 * (double)2) + (k3 * (double)2) + k4) * (dt / 6));
}

/**
 * @brief Tranforms the given state space vector into measurement space.
 */
measurement_vector_t UKF::state_to_measurement(state_vector_t vector)
{
  measurement_vector_t m;
  m(0, 0) = vector(0, 0);
  m(1, 0) = vector(1, 0);
  return m;
}

void UKF::set_speed(double speed)
{
  this->speed = speed;
}

void UKF::set_gps_noise(double accuracy)
{
  // Convert mm to m
  accuracy /= 1000.0;
  // Serial.printf("ACCURACY: %f\n", accuracy);

  double sigma = (accuracy / (0.848867684498)) * (accuracy / (0.848867684498));
  this->gps_noise = measurement_cov_matrix_t{{sigma, 0}, {0, sigma}};
}

void UKF::predict(input_vector_t input, double dt)
{
  // Serial.printf("dt: %f\n", dt);
  if (abs(this->speed) > MOVING_THRESHOLD)
  {
    state_vector_t state_sigmas[2 * STATE_SPACE_DIM + 1];
    double state_weights[2 * STATE_SPACE_DIM + 1];
    this->generate_sigmas(this->curr_state_est, this->curr_state_cov, state_sigmas, state_weights);

    for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
    {
      state_sigmas[i] = rk4(state_sigmas[i], input, dt);
      // Serial.printf("State sigma %d: %f, %f, %f\n", i, state_sigmas[i](0, 0), state_sigmas[i](1, 0), state_sigmas[i](2, 0));
    }

    this->curr_state_est.fill(0);
    this->curr_state_cov.fill(0);
    for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
    {
      this->curr_state_est += state_sigmas[i] * state_weights[i];
    }

    for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
    {
      state_vector_t m = state_sigmas[i] - this->curr_state_est;
      this->curr_state_cov += ((m * m.transpose()) * state_weights[i]);
    }

    this->curr_state_cov += this->process_noise * dt;
  }
}

void UKF::update(measurement_vector_t measurement)
{
  state_vector_t state_sigmas[2 * STATE_SPACE_DIM + 1];
  double weights[2 * STATE_SPACE_DIM + 1];
  this->generate_sigmas(this->curr_state_est, this->curr_state_cov, state_sigmas, weights);

  measurement_vector_t measurement_sigmas[2 * STATE_SPACE_DIM + 1];
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    measurement_sigmas[i] = state_to_measurement(state_sigmas[i]);
    // Serial.printf("State sigma %d: %f, %f, %f\n", i, state_sigmas[i](0, 0), state_sigmas[i](1, 0), state_sigmas[i](2, 0));
    // Serial.printf("Measurement sigma %d: %f, %f\n", i, measurement_sigmas[i](0, 0), measurement_sigmas[i](1, 0));
  }

  measurement_vector_t predicted_measurement;
  predicted_measurement.fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    predicted_measurement += measurement_sigmas[i] * weights[i];
  }

  measurement_cov_matrix_t innovation_cov;
  Eigen::Matrix<double, STATE_SPACE_DIM, MEASUREMENT_SPACE_DIM> cross_cov;
  innovation_cov.fill(0);
  cross_cov.fill(0);
  for (int i = 0; i < 2 * STATE_SPACE_DIM + 1; i++)
  {
    measurement_vector_t m = measurement_sigmas[i] - predicted_measurement;
    innovation_cov += (m * m.transpose()) * weights[i];
    cross_cov += ((state_sigmas[i] - this->curr_state_est) * m.transpose()) * weights[i];
  }
  innovation_cov += this->gps_noise;

  Eigen::Matrix<double, STATE_SPACE_DIM, MEASUREMENT_SPACE_DIM> kalman_gain = cross_cov * innovation_cov.inverse();

  // Serial.printf("Measurement: %f, %f\n", measurement(0, 0), measurement(1, 0));
  // Serial.printf("Predicted measurement: %f, %f\n", predicted_measurement(0, 0), predicted_measurement(1, 0));
  // Serial.printf("Kalman gain:\n%f,%f\n%f,%f\n%f,%f\n", kalman_gain(0, 0), kalman_gain(0, 1), kalman_gain(1, 0), kalman_gain(1, 1), kalman_gain(2, 0), kalman_gain(2, 1));
  this->curr_state_est += (kalman_gain * (measurement - predicted_measurement));
  this->curr_state_cov -= (kalman_gain * (innovation_cov * kalman_gain.transpose()));
}

#if 0

void run_kalman() {
#if 0
  /**************** Kalman Filtering! ****************/
  // output file setup
  File filter_out = SD.open("filter.csv", FILE_WRITE);
  filter_out.println("timestamp,pos_x,pos_y,heading");
  File cov_out = SD.open("covariance.csv", FILE_WRITE);
  cov_out.println("timestamp,c1,c2,c3,c4,c5,c6,c7,c8,c9");
#endif

  // TODO in real buggy code, set initial x and y first gps reading
  // set heading to where it's facing in tent upon power on, probably
  curr_state_est(0, 0) = stream.next_gps_x;
  curr_state_est(1, 0) = stream.next_gps_y;
  // curr_state_est(2, 0) = /* ???? */

  double last_predict_timestamp = stream.steering_time();
  // running filter!
  while (1)
  {
    static int point = 0;

    uint32_t diff0 = micros() - mic0;
    //Serial.printf("filter took %lu micros\n", diff0);

    uint32_t mic = micros();
    stream.advance(m);
    uint32_t diff = micros() - mic;
    //Serial.printf("advance took %lu micros\n", diff);

    uint32_t mic2 = micros();
    filter_out.printf("%f,%f,%f,%f\n", last_predict_timestamp, updated_state_est(0, 0), updated_state_est(1, 0), updated_state_est(2, 0));
    cov_out.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", last_predict_timestamp, updated_state_cov(0, 0), updated_state_cov(0, 1), updated_state_cov(0, 2),
                   updated_state_cov(1, 0), updated_state_cov(1, 1), updated_state_cov(1, 2),
                   updated_state_cov(2, 0), updated_state_cov(2, 1), updated_state_cov(2, 2));


    ++point;

    static int i = 0;
    if (++i >= 1000) {
      i = 0;
      filter_out.flush();
      cov_out.flush();

      Serial.println("===========================\n");
      Serial.printf("Data point %d, time %f\n", point, last_predict_timestamp);
      Serial.println("===========================\n");
    }
    uint32_t diff2 = micros() - mic2;
    //Serial.printf("printf took %lu micros\n", diff2);

  }

  filter_out.close();
  cov_out.close();
  run = false;
  Serial.println("FILTER DONE");
}

#endif