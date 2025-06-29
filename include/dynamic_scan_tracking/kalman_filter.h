#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include <vector>

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void predict(float delta_T);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void update(const Eigen::VectorXd &z);

  /**
   * Predicts the state by using Extended Kalman Filter equations (CTRV model)
   * @param delta_T Time between k and k+1 in s
   */
  void predictCTRV(float delta_T);

  /**
   * Predicts the state using adaptive CV/CTRV model selection with estimated turn rate
   * @param delta_T Time between k and k+1 in s
   */
  void predictAdaptiveModel(float delta_T);

  /**
   * Set turn rate for CTRV model
   * @param turn_rate Turn rate in rad/s
   */
  void setTurnRate(float turn_rate);

  /**
   * Set process noise parameters for CTRV model
   * @param noise_a Acceleration noise (m/s²)
   * @param noise_yaw_accel Yaw acceleration noise (rad/s²)
   */
  void setProcessNoise(float noise_a, float noise_yaw_accel);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  // process noise variable for Kalman filter Q
  float q_noise;
  
  // CTRV model parameters
  float turn_rate_;       // Default turn rate for CTRV model (rad/s)
  float noise_a_;         // Acceleration noise (m/s²)
  float noise_yaw_accel_; // Yaw acceleration noise (rad/s²)
  
  // Adaptive turn rate estimation
  std::vector<double> velocity_history_x_;
  std::vector<double> velocity_history_y_;
  std::vector<double> time_history_;
  static const int MAX_HISTORY_SIZE = 10;  // Keep last 10 velocity measurements
  double estimated_turn_rate_;
  double turn_rate_confidence_;
  
  // Motion model selection
  bool use_adaptive_model_;
  
  /**
   * Estimate current turn rate from velocity history
   * @return Estimated turn rate in rad/s
   */
  double estimateTurnRate();
  
  /**
   * Update velocity history for turn rate estimation
   * @param vx Current X velocity
   * @param vy Current Y velocity
   * @param timestamp Current time
   */
  void updateVelocityHistory(double vx, double vy, double timestamp);
  
  /**
   * Set whether to use adaptive model selection
   * @param use_adaptive True to use adaptive CV/CTRV selection
   */
  void setUseAdaptiveModel(bool use_adaptive);
};

#endif // KALMAN_FILTER_H_