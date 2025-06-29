#ifndef INTEGRATION_TIME_MANAGER_H
#define INTEGRATION_TIME_MANAGER_H

#include <Eigen/Dense>
#include <cmath>

class IntegrationTimeManager 
{
public:
    IntegrationTimeManager(int min_integration_time_ast, int max_integration_time_ast,
                          int min_integration_time_adt, int max_integration_time_adt,
                          float max_distance_ast, float max_distance_adt);
    ~IntegrationTimeManager() = default;

    /**
     * @brief Adjust integration time based on object distance
     * @param min_integration_time Minimum integration time
     * @param max_distance Maximum distance for this tracker
     * @param distance Current object distance (output)
     * @param max_scans Maximum number of scans
     * @param optimal_integration_time Current optimal integration time (input/output)
     * @param pose Current pose estimate
     */
    void adjustIntegrationTime(int min_integration_time, float max_distance,
                              float& distance, int max_scans,
                              int& optimal_integration_time,
                              const Eigen::VectorXd& pose);

    /**
     * @brief Get optimal integration times for both AST and ADT
     * @param pose_fused Fused pose estimate
     * @param optimal_time_ast Output optimal AST integration time
     * @param optimal_time_adt Output optimal ADT integration time
     * @param distance_ast Output AST distance
     * @param distance_adt Output ADT distance
     */
    void getOptimalIntegrationTimes(const Eigen::VectorXd& pose_fused,
                                   int& optimal_time_ast, int& optimal_time_adt,
                                   float& distance_ast, float& distance_adt);

private:
    int min_integration_time_ast_;
    int max_integration_time_ast_;
    int min_integration_time_adt_;
    int max_integration_time_adt_;
    float max_distance_ast_;
    float max_distance_adt_;
};

#endif // INTEGRATION_TIME_MANAGER_H
