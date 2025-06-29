#include "dynamic_scan_tracking/integration_time_manager.h"
#include <algorithm>
#include <cmath>

IntegrationTimeManager::IntegrationTimeManager(int min_integration_time_ast, int max_integration_time_ast,
                                              int min_integration_time_adt, int max_integration_time_adt,
                                              float max_distance_ast, float max_distance_adt)
    : min_integration_time_ast_(min_integration_time_ast)
    , max_integration_time_ast_(max_integration_time_ast)
    , min_integration_time_adt_(min_integration_time_adt)
    , max_integration_time_adt_(max_integration_time_adt)
    , max_distance_ast_(max_distance_ast)
    , max_distance_adt_(max_distance_adt)
{
}

void IntegrationTimeManager::adjustIntegrationTime(int min_integration_time, float max_distance,
                                                  float& distance, int max_scans,
                                                  int& optimal_integration_time,
                                                  const Eigen::VectorXd& pose)
{
    // Compute object distance
    distance = sqrt(pose[0] * pose[0] + pose[1] * pose[1] + pose[2] * pose[2]);

    // Compute new optimal integration time based on distance
    optimal_integration_time = std::round(min_integration_time +
                                         (max_scans - min_integration_time) *
                                         (distance / max_distance));

    // Clamp to valid range
    optimal_integration_time = std::max(min_integration_time, 
                                       std::min(max_scans, optimal_integration_time));
}

void IntegrationTimeManager::getOptimalIntegrationTimes(const Eigen::VectorXd& pose_fused,
                                                       int& optimal_time_ast, int& optimal_time_adt,
                                                       float& distance_ast, float& distance_adt)
{
    // Adjust AST integration time
    adjustIntegrationTime(min_integration_time_ast_, max_distance_ast_, distance_ast,
                         max_integration_time_ast_, optimal_time_ast, pose_fused);

    // Adjust ADT integration time
    adjustIntegrationTime(min_integration_time_adt_, max_distance_adt_, distance_adt,
                         max_integration_time_adt_, optimal_time_adt, pose_fused);
}
