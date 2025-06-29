#include <dynamic_scan_tracking/kalman_filter.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <iomanip>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() : turn_rate_(0.1), noise_a_(0.5), noise_yaw_accel_(0.1), 
                               estimated_turn_rate_(0.0), turn_rate_confidence_(0.0), 
                               use_adaptive_model_(true) {} // Default values

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::predict(float delta_T)
{

  // Update state transition matrix F for a constant velocity motion model
  F_.setIdentity();
  F_(0, 3) = delta_T;
  F_(1, 4) = delta_T;
  F_(2, 5) = delta_T;

  // Update process noise covariance matrix Q for a constant velocity motion model
  Q_.setZero();
  
  // Use configurable acceleration noise instead of hardcoded value
  float noise_a = noise_a_;
  double dt2 = delta_T * delta_T;
  double dt3 = dt2 * delta_T;
  double dt4 = dt3 * delta_T;
  
  // Position uncertainty from acceleration
  Q_(0, 0) = dt4/4 * noise_a * noise_a;
  Q_(1, 1) = dt4/4 * noise_a * noise_a;
  Q_(2, 2) = dt4/4 * noise_a * noise_a * 0.5; // Less vertical noise for UAVs
  
  // Position-velocity correlation
  Q_(0, 3) = dt3/2 * noise_a * noise_a;
  Q_(1, 4) = dt3/2 * noise_a * noise_a;
  Q_(2, 5) = dt3/2 * noise_a * noise_a * 0.5;
  Q_(3, 0) = Q_(0, 3);
  Q_(4, 1) = Q_(1, 4);
  Q_(5, 2) = Q_(2, 5);
  
  // Velocity uncertainty
  Q_(3, 3) = dt2 * noise_a * noise_a;
  Q_(4, 4) = dt2 * noise_a * noise_a;
  Q_(5, 5) = dt2 * noise_a * noise_a * 0.5;

  x_ = F_ * x_; // predict new estimate
  P_ = F_ * P_ * F_.transpose() + Q_; // update new state estimation covariance matrix

}

void KalmanFilter::predictCTRV(float delta_T)
{
    // CTRV (Constant Turn Rate and Velocity) Model for UAV tracking
    // State vector: [x, y, z, vx, vy, vz]
    // This model assumes the UAV can change direction but maintains relatively constant speed
    
    // Extract current state
    double px = x_(0);
    double py = x_(1);
    double pz = x_(2);
    double vx = x_(3);
    double vy = x_(4);
    double vz = x_(5);
    
    // Calculate current speed and heading for CTRV
    double v = sqrt(vx*vx + vy*vy);
    double yaw = atan2(vy, vx);
    
    // Use configurable turn rate
    double yaw_rate = turn_rate_;
    
    // CTRV prediction equations
    if (fabs(yaw_rate) > 0.001) {
        // Non-zero turn rate case
        x_(0) = px + (v/yaw_rate) * (sin(yaw + yaw_rate*delta_T) - sin(yaw));
        x_(1) = py + (v/yaw_rate) * (-cos(yaw + yaw_rate*delta_T) + cos(yaw));
        x_(3) = v * cos(yaw + yaw_rate*delta_T);
        x_(4) = v * sin(yaw + yaw_rate*delta_T);
    } else {
        // Zero turn rate case (straight line motion)
        x_(0) = px + vx * delta_T;
        x_(1) = py + vy * delta_T;
        x_(3) = vx;
        x_(4) = vy;
    }
    
    // Z-axis follows simple constant velocity (UAVs typically have simpler vertical motion)
    x_(2) = pz + vz * delta_T;
    x_(5) = vz;
    
    // Compute proper Jacobian for CTRV model
    F_.setIdentity();
    
    if (v > 0.001) { // Avoid division by zero
        if (fabs(yaw_rate) > 0.001) {
            // Partial derivatives for turning motion
            double sin_yaw = sin(yaw);
            double cos_yaw = cos(yaw);
            double sin_yaw_dt = sin(yaw + yaw_rate * delta_T);
            double cos_yaw_dt = cos(yaw + yaw_rate * delta_T);
            
            // ∂px_new/∂vx, ∂px_new/∂vy
            F_(0, 3) = (sin_yaw_dt - sin_yaw) / yaw_rate * vx / v;
            F_(0, 4) = (sin_yaw_dt - sin_yaw) / yaw_rate * vy / v;
            
            // ∂py_new/∂vx, ∂py_new/∂vy  
            F_(1, 3) = (-cos_yaw_dt + cos_yaw) / yaw_rate * vx / v;
            F_(1, 4) = (-cos_yaw_dt + cos_yaw) / yaw_rate * vy / v;
            
            // ∂vx_new/∂vx, ∂vx_new/∂vy
            F_(3, 3) = cos_yaw_dt * vx / v;
            F_(3, 4) = cos_yaw_dt * vy / v;
            
            // ∂vy_new/∂vx, ∂vy_new/∂vy
            F_(4, 3) = sin_yaw_dt * vx / v;
            F_(4, 4) = sin_yaw_dt * vy / v;
        } else {
            // Linear motion Jacobian
            F_(0, 3) = delta_T;
            F_(1, 4) = delta_T;
        }
    }
    
    // Z-axis is always linear
    F_(2, 5) = delta_T;
    
    // Process noise matrix for CTRV (using configurable parameters)
    float noise_a = noise_a_;           // Configurable acceleration noise
    float noise_yaw_accel = noise_yaw_accel_; // Configurable yaw acceleration noise
    
    Q_.setZero();
    
    // Position uncertainty from acceleration
    double dt2 = delta_T * delta_T;
    double dt3 = dt2 * delta_T;
    double dt4 = dt3 * delta_T;
    
    Q_(0, 0) = dt4/4 * noise_a * noise_a;
    Q_(1, 1) = dt4/4 * noise_a * noise_a;
    Q_(2, 2) = dt4/4 * noise_a * noise_a * 0.5; // Less vertical noise
    
    // Position-velocity correlation
    Q_(0, 3) = dt3/2 * noise_a * noise_a;
    Q_(1, 4) = dt3/2 * noise_a * noise_a;
    Q_(2, 5) = dt3/2 * noise_a * noise_a * 0.5;
    Q_(3, 0) = Q_(0, 3);
    Q_(4, 1) = Q_(1, 4);
    Q_(5, 2) = Q_(2, 5);
    
    // Velocity uncertainty
    Q_(3, 3) = dt2 * noise_a * noise_a;
    Q_(4, 4) = dt2 * noise_a * noise_a;
    Q_(5, 5) = dt2 * noise_a * noise_a * 0.5;
    
    // Coupling for CTRV (turn rate affects x-y motion)
    Q_(0, 1) = Q_(1, 0) = dt4/8 * noise_yaw_accel * noise_yaw_accel;
    Q_(3, 4) = Q_(4, 3) = dt2/2 * noise_yaw_accel * noise_yaw_accel;
    
    // Update covariance with proper Jacobian
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const VectorXd &z)
{
  // Input validation
  if (z.size() != H_.rows()) {
    std::cerr << "KalmanFilter::update: Measurement dimension mismatch!" << std::endl;
    return;
  }

  VectorXd z_pred = H_ * x_; // compute predicted observation
  VectorXd y = z - z_pred; // residual

  MatrixXd S = H_ * P_ * H_.transpose() + R_;

  // Use LLT decomposition for numerical stability
  Eigen::LLT<Eigen::MatrixXd> llt_S(S);
  if (llt_S.info() == Eigen::Success) {
    MatrixXd K = P_ * H_.transpose() * llt_S.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    
    // Estimate new state
    x_ = x_ + K * y;

    // Joseph form update for numerical stability and positive definiteness
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    MatrixXd IKH = I - K * H_;
    P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();
  } else {
    // Fallback to standard implementation if LLT fails
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;
  }
  
  // Ensure covariance remains positive definite
  P_ = 0.5 * (P_ + P_.transpose());
}

void KalmanFilter::setTurnRate(float turn_rate) {
    turn_rate_ = turn_rate;
}

void KalmanFilter::setProcessNoise(float noise_a, float noise_yaw_accel) {
    noise_a_ = noise_a;
    noise_yaw_accel_ = noise_yaw_accel;
}

void KalmanFilter::updateVelocityHistory(double vx, double vy, double timestamp)
{
    velocity_history_x_.push_back(vx);
    velocity_history_y_.push_back(vy);
    time_history_.push_back(timestamp);
    
    // Keep only recent history
    if (velocity_history_x_.size() > MAX_HISTORY_SIZE) {
        velocity_history_x_.erase(velocity_history_x_.begin());
        velocity_history_y_.erase(velocity_history_y_.begin());
        time_history_.erase(time_history_.begin());
    }
}

double KalmanFilter::estimateTurnRate()
{
    if (velocity_history_x_.size() < 3) {
        // Not enough history, return zero turn rate (use CV model)
        turn_rate_confidence_ = 0.0;
        return 0.0;
    }
    
    // Calculate angular velocity from velocity vector changes
    std::vector<double> angular_velocities;
    
    for (size_t i = 1; i < velocity_history_x_.size(); i++) {
        double vx_prev = velocity_history_x_[i-1];
        double vy_prev = velocity_history_y_[i-1];
        double vx_curr = velocity_history_x_[i];
        double vy_curr = velocity_history_y_[i];
        double dt = time_history_[i] - time_history_[i-1];
        
        if (dt > 0.001) { // Avoid division by zero
            // Calculate heading change
            double heading_prev = atan2(vy_prev, vx_prev);
            double heading_curr = atan2(vy_curr, vx_curr);
            
            // Handle angle wrapping
            double heading_diff = heading_curr - heading_prev;
            while (heading_diff > M_PI) heading_diff -= 2*M_PI;
            while (heading_diff < -M_PI) heading_diff += 2*M_PI;
            
            double angular_vel = heading_diff / dt;
            angular_velocities.push_back(angular_vel);
        }
    }
    
    if (angular_velocities.empty()) {
        turn_rate_confidence_ = 0.0;
        return 0.0;
    }
    
    // Calculate mean and variance of angular velocities
    double mean_angular_vel = 0.0;
    for (double av : angular_velocities) {
        mean_angular_vel += av;
    }
    mean_angular_vel /= angular_velocities.size();
    
    // Calculate variance to assess consistency
    double variance = 0.0;
    for (double av : angular_velocities) {
        variance += (av - mean_angular_vel) * (av - mean_angular_vel);
    }
    variance /= angular_velocities.size();
    
    // Set confidence based on consistency (low variance = high confidence)
    // Also consider magnitude (very small turn rates are likely noise)
    double magnitude_confidence = std::min(1.0, fabs(mean_angular_vel) / 0.1); // 0.1 rad/s threshold
    double consistency_confidence = exp(-variance * 10.0); // Exponential decay with variance
    turn_rate_confidence_ = magnitude_confidence * consistency_confidence;
    
    return mean_angular_vel;
}

void KalmanFilter::setUseAdaptiveModel(bool use_adaptive)
{
    use_adaptive_model_ = use_adaptive;
}

void KalmanFilter::predictAdaptiveModel(float delta_T)
{
    if (!use_adaptive_model_) {
        // Fall back to regular CV prediction
        predict(delta_T);
        return;
    }
    
    // Update velocity history with current state
    double current_time = time_history_.empty() ? 0.0 : time_history_.back() + delta_T;
    updateVelocityHistory(x_(3), x_(4), current_time);
    
    // Estimate current turn rate
    estimated_turn_rate_ = estimateTurnRate();
    
    // Store original states for blending
    VectorXd x_cv = x_;
    MatrixXd P_cv = P_;
    VectorXd x_ctrv = x_;
    MatrixXd P_ctrv = P_;
    
    // Predict with CV model
    predict(delta_T);
    x_cv = x_;
    P_cv = P_;
    
    // Reset state and predict with CTRV model using estimated turn rate
    x_ = x_ctrv; // Reset to original state
    P_ = P_ctrv;
    
    // Temporarily set turn rate to estimated value
    float original_turn_rate = turn_rate_;
    turn_rate_ = estimated_turn_rate_;
    
    predictCTRV(delta_T);
    x_ctrv = x_;
    P_ctrv = P_;
    
    // Restore original turn rate
    turn_rate_ = original_turn_rate;
    
    // Blend predictions based on turn rate confidence
    // High confidence in turn rate -> use more CTRV
    // Low confidence -> use more CV
    double cv_weight = 1.0 - turn_rate_confidence_;
    double ctrv_weight = turn_rate_confidence_;
    
    // Apply minimum weights to ensure both models contribute
    cv_weight = std::max(0.1, cv_weight);
    ctrv_weight = std::max(0.1, ctrv_weight);
    
    // Normalize weights
    double total_weight = cv_weight + ctrv_weight;
    cv_weight /= total_weight;
    ctrv_weight /= total_weight;
    
    // Blend state estimates
    x_ = cv_weight * x_cv + ctrv_weight * x_ctrv;
    
    // Blend covariance matrices (simplified approximation)
    P_ = cv_weight * P_cv + ctrv_weight * P_ctrv;
    
    // Add some additional uncertainty due to model mixing
    MatrixXd model_uncertainty = MatrixXd::Identity(6, 6) * 0.01;
    P_ += model_uncertainty;
    
    // Diagnostic output to verify adaptive model is working
    static int debug_counter = 0;
    if (++debug_counter % 20 == 0) { // Print every 20 calls to avoid spam
        std::cout << "[ADAPTIVE] Turn Rate: " << std::fixed << std::setprecision(3) << estimated_turn_rate_ 
                  << " rad/s, Confidence: " << turn_rate_confidence_ 
                  << ", Weights: CV=" << cv_weight << " CTRV=" << ctrv_weight 
                  << ", History: " << velocity_history_x_.size() << " samples" << std::endl;
    }
}