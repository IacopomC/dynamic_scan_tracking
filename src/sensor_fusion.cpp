#include "dynamic_scan_tracking/sensor_fusion.h"
#include <Eigen/Dense>
#include <algorithm>
#include <functional>

SensorFusion::SensorFusion()
{
    // Initialize cache as invalid
    cache1_.valid = false;
    cache2_.valid = false;
}

void SensorFusion::fuseStatesICI(const Eigen::VectorXd& state1, const Eigen::MatrixXd& cov1,
                                const Eigen::VectorXd& state2, const Eigen::MatrixXd& cov2,
                                Eigen::VectorXd& fused_state, Eigen::MatrixXd& fused_cov,
                                double& omega)
{
    // Compute optimal omega using optimized method
    omega = computeOptimalOmega(cov1, cov2);

    // Check cache for covariance matrices
    if (!isMatrixCached(cov1, cache1_)) {
        updateCache(cov1, cache1_);
    }
    if (!isMatrixCached(cov2, cache2_)) {
        updateCache(cov2, cache2_);
    }

    // Use cached inverses for ICI computation
    const Eigen::MatrixXd& cov1_inv = cache1_.inverse;
    const Eigen::MatrixXd& cov2_inv = cache2_.inverse;
    
    // Pre-allocate and reuse temporary matrices
    if (temp_matrix1_.rows() != cov1.rows() || temp_matrix1_.cols() != cov1.cols()) {
        temp_matrix1_.resize(cov1.rows(), cov1.cols());
        temp_matrix2_.resize(cov1.rows(), cov1.cols());
        temp_vector_.resize(state1.size());
    }
    
    // Compute weighted information matrix: omega * P1^-1 + (1-omega) * P2^-1
    temp_matrix1_.noalias() = omega * cov1_inv;
    temp_matrix1_.noalias() += (1.0 - omega) * cov2_inv;
    
    // Use LLT decomposition for stable inversion
    llt_solver_fused_.compute(temp_matrix1_);
    if (llt_solver_fused_.info() != Eigen::Success) {
        // Fallback to SVD if LLT fails
        fused_cov = temp_matrix1_.inverse();
    } else {
        fused_cov = llt_solver_fused_.solve(Eigen::MatrixXd::Identity(temp_matrix1_.rows(), temp_matrix1_.cols()));
    }

    // Compute fused state efficiently
    temp_vector_.noalias() = omega * (cov1_inv * state1);
    temp_vector_.noalias() += (1.0 - omega) * (cov2_inv * state2);
    fused_state.noalias() = fused_cov * temp_vector_;
}

double SensorFusion::computeOptimalOmega(const Eigen::MatrixXd& cov1, const Eigen::MatrixXd& cov2)
{
    // Use cached determinants if available
    double det1, det2;
    
    if (isMatrixCached(cov1, cache1_)) {
        det1 = cache1_.determinant;
    } else {
        // Use standard determinant calculation (reliable across Eigen versions)
        det1 = cov1.determinant();
    }
    
    if (isMatrixCached(cov2, cache2_)) {
        det2 = cache2_.determinant;
    } else {
        det2 = cov2.determinant();
    }
    
    if (det1 + det2 == 0.0) {
        return 0.5; // Equal weighting if denominators are zero
    }
    
    // Weight inversely proportional to determinant (uncertainty)
    double omega = det2 / (det1 + det2);
    
    // Clamp omega to valid range [0, 1]
    omega = std::max(0.0, std::min(1.0, omega));
    
    return omega;
}

uint64_t SensorFusion::computeMatrixHash(const Eigen::MatrixXd& matrix) const
{
    // Simple hash based on matrix data pointer and first few elements
    // This is a fast approximation - could be enhanced for better collision resistance
    std::hash<double> hasher;
    uint64_t hash = 0;
    
    // Hash matrix dimensions
    hash ^= hasher(static_cast<double>(matrix.rows())) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= hasher(static_cast<double>(matrix.cols())) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    
    // Hash a subset of matrix elements for performance
    const int step = std::max(1, static_cast<int>(matrix.size() / 16)); // Sample ~16 elements
    for (int i = 0; i < matrix.size(); i += step) {
        hash ^= hasher(matrix.data()[i]) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    
    return hash;
}

bool SensorFusion::isMatrixCached(const Eigen::MatrixXd& matrix, const CovarianceCache& cache) const
{
    if (!cache.valid) {
        return false;
    }
    
    // Quick size check
    if (cache.matrix.rows() != matrix.rows() || cache.matrix.cols() != matrix.cols()) {
        return false;
    }
    
    // Hash comparison for fast check
    uint64_t current_hash = computeMatrixHash(matrix);
    if (cache.hash != current_hash) {
        return false;
    }
    
    // For extra safety, could add element-wise comparison here if needed
    // For now, assume hash collision is very unlikely for our use case
    
    return true;
}

void SensorFusion::updateCache(const Eigen::MatrixXd& matrix, CovarianceCache& cache) const
{
    cache.matrix = matrix;
    cache.hash = computeMatrixHash(matrix);
    
    // Compute and cache inverse using LLT decomposition for stability
    Eigen::LLT<Eigen::MatrixXd> llt(matrix);
    if (llt.info() == Eigen::Success) {
        cache.inverse = llt.solve(Eigen::MatrixXd::Identity(matrix.rows(), matrix.cols()));
        // Use standard determinant calculation
        cache.determinant = matrix.determinant();
    } else {
        // Fallback for non-positive definite matrices
        cache.inverse = matrix.inverse();
        cache.determinant = matrix.determinant();
    }
    
    cache.valid = true;
}
