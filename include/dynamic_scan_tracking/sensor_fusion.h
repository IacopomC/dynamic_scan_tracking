#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Eigen/Dense>
#include "dynamic_scan_tracking/kalman_filter.h"

class SensorFusion 
{
public:
    SensorFusion();
    ~SensorFusion() = default;

    /**
     * @brief Fuse two Kalman filter estimates using Intersection Covariance Intersection (ICI)
     * @param state1 First Kalman filter state
     * @param cov1 First Kalman filter covariance
     * @param state2 Second Kalman filter state
     * @param cov2 Second Kalman filter covariance
     * @param fused_state Output fused state
     * @param fused_cov Output fused covariance
     * @param omega Weight parameter for fusion
     */
    void fuseStatesICI(const Eigen::VectorXd& state1, const Eigen::MatrixXd& cov1,
                      const Eigen::VectorXd& state2, const Eigen::MatrixXd& cov2,
                      Eigen::VectorXd& fused_state, Eigen::MatrixXd& fused_cov,
                      double& omega);

    /**
     * @brief Compute optimal omega weight for ICI fusion
     * @param cov1 First covariance matrix
     * @param cov2 Second covariance matrix
     * @return Optimal omega value
     */
    double computeOptimalOmega(const Eigen::MatrixXd& cov1, const Eigen::MatrixXd& cov2);

private:
    // Pre-allocated matrices for performance
    mutable Eigen::MatrixXd temp_matrix1_;
    mutable Eigen::MatrixXd temp_matrix2_;
    mutable Eigen::MatrixXd temp_matrix3_;
    mutable Eigen::VectorXd temp_vector_;
    
    // LLT decomposition objects for reuse
    mutable Eigen::LLT<Eigen::MatrixXd> llt_solver1_;
    mutable Eigen::LLT<Eigen::MatrixXd> llt_solver2_;
    mutable Eigen::LLT<Eigen::MatrixXd> llt_solver_fused_;
    
    // Cache for covariance properties
    struct CovarianceCache {
        Eigen::MatrixXd matrix;
        Eigen::MatrixXd inverse;
        double determinant;
        bool valid;
        uint64_t hash;
    };
    
    mutable CovarianceCache cache1_;
    mutable CovarianceCache cache2_;
    
    // Utility functions
    uint64_t computeMatrixHash(const Eigen::MatrixXd& matrix) const;
    bool isMatrixCached(const Eigen::MatrixXd& matrix, const CovarianceCache& cache) const;
    void updateCache(const Eigen::MatrixXd& matrix, CovarianceCache& cache) const;
};

#endif // SENSOR_FUSION_H
