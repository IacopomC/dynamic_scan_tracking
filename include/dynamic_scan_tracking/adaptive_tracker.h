#ifndef ADAPTIVE_TRACKER_H
#define ADAPTIVE_TRACKER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include "dynamic_scan_tracking/kalman_filter.h"

typedef pcl::PointXYZINormal PointType;

class AdaptiveTracker 
{
public:
    AdaptiveTracker(float search_radius, float gamma, const std::string& frame_id);
    ~AdaptiveTracker() = default;

    /**
     * @brief Extract object cloud from integrated point cloud using KDTree
     * @param point_cloud Input integrated point cloud
     * @param object_cloud Output object point cloud
     * @param future_position Predicted position for search center
     * @param kalman_filter Kalman filter for prediction
     * @param search_radius Radius for KDTree search
     */
    void extractObjectCloud(pcl::PointCloud<PointType>::Ptr& point_cloud,
                           pcl::PointCloud<PointType>::Ptr& object_cloud,
                           PointType& future_position,
                           KalmanFilter& kalman_filter,
                           float search_radius);

    /**
     * @brief Update object position using weighted average of detected points
     * @param object_cloud Input object point cloud
     * @param kalman_filter Kalman filter to update
     * @param msg_timebase_ns Message timebase for weight calculation
     * @param delta_t Time delta for prediction
     */
    void updatePosition(pcl::PointCloud<PointType>::Ptr& object_cloud,
                       KalmanFilter& kalman_filter,
                       unsigned long msg_timebase_ns,
                       double delta_t);

    /**
     * @brief Set the motion model type for prediction
     * @param model_type Motion model type: "cv", "ctrv", or "adaptive"
     */
    void setMotionModelType(const std::string& model_type);

    /**
     * @brief Compute pose from Kalman filter state
     * @param current_pose Output current pose
     * @param previous_pose Previous pose for orientation calculation (input/output)
     * @param state Kalman filter state vector
     */
    void computePose(geometry_msgs::PoseStamped& current_pose,
                    geometry_msgs::PoseStamped& previous_pose,
                    const Eigen::VectorXd& state);

private:
    float search_radius_;
    float gamma_;  // Weight decay constant for temporal weighting
    std::string frame_id_;  // Configurable frame ID
    std::string motion_model_type_;  // Motion model type: "cv", "ctrv", or "adaptive"
    
    // Pre-allocated objects for performance
    pcl::KdTreeFLANN<PointType>::Ptr kd_tree_;
    std::vector<int> point_indices_;
    std::vector<float> point_distances_;
    
    // Track if we have a valid previous pose for orientation calculation
    bool has_previous_pose_;
};

#endif // ADAPTIVE_TRACKER_H
