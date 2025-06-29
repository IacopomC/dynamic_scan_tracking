#include "dynamic_scan_tracking/adaptive_tracker.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <cmath>

AdaptiveTracker::AdaptiveTracker(float search_radius, float gamma, const std::string& frame_id)
    : search_radius_(search_radius), gamma_(gamma), frame_id_(frame_id), motion_model_type_("adaptive"), has_previous_pose_(false)
{
    // Pre-allocate KDTree and search vectors for performance
    kd_tree_ = pcl::KdTreeFLANN<PointType>::Ptr(new pcl::KdTreeFLANN<PointType>());
    point_indices_.reserve(1000);    // Pre-reserve capacity
    point_distances_.reserve(1000);
}

void AdaptiveTracker::extractObjectCloud(pcl::PointCloud<PointType>::Ptr& point_cloud,
                                        pcl::PointCloud<PointType>::Ptr& object_cloud,
                                        PointType& future_position,
                                        KalmanFilter& kalman_filter,
                                        float search_radius)
{
    // Input validation for error handling
    if (!point_cloud || point_cloud->empty()) {
        ROS_WARN("AdaptiveTracker: Empty or null input point cloud");
        return;
    }
    
    if (!object_cloud) {
        ROS_ERROR("AdaptiveTracker: Null output point cloud pointer");
        return;
    }

    // Reuse pre-allocated KDTree instead of creating new one each time
    kd_tree_->setInputCloud(point_cloud);

    // Clear and reuse pre-allocated search vectors
    point_indices_.clear();
    point_distances_.clear();

    // Clear object cloud of previous message data
    object_cloud->clear();

    // Define object point cloud's timestamp and frame id
    object_cloud->header.frame_id = frame_id_;
    object_cloud->header.stamp = ros::Time::now().toNSec() / 1e3;

    // Calculate predicted position using kalman filter
    // Use future position in KDTree to find object in point cloud
    future_position.x = kalman_filter.x_[0];
    future_position.y = kalman_filter.x_[1];
    future_position.z = kalman_filter.x_[2];

    // If neighbors within radius are found
    if (kd_tree_->radiusSearch(future_position, search_radius, point_indices_, point_distances_) > 0)
    {
        // Reserve space for efficiency
        object_cloud->points.reserve(point_indices_.size());
        
        for (std::size_t i = 0; i < point_indices_.size(); ++i)
        {
            // Bounds checking for safety
            if (point_indices_[i] >= 0 && point_indices_[i] < static_cast<int>(point_cloud->size()))
            {
                // Retrieve point belonging to object from point cloud based on index
                const PointType& pt = (*point_cloud)[point_indices_[i]];

                // Add point to object point cloud
                object_cloud->points.push_back(pt);
            }
        }
    }
}

void AdaptiveTracker::setMotionModelType(const std::string& model_type)
{
    motion_model_type_ = model_type;
    ROS_DEBUG("AdaptiveTracker: Motion model type set to %s", model_type.c_str());
}

void AdaptiveTracker::updatePosition(pcl::PointCloud<PointType>::Ptr& object_cloud,
                                    KalmanFilter& kalman_filter,
                                    unsigned long msg_timebase_ns,
                                    double delta_t)
{
    // Input validation
    if (!object_cloud) {
        ROS_ERROR("AdaptiveTracker: Null object cloud in updatePosition");
        return;
    }

    // ALWAYS predict using Kalman filter (this is crucial for tracking through occlusions)
    static int debug_counter = 0;
    if (++debug_counter % 50 == 0) { // Print every 50 calls
        ROS_INFO("AdaptiveTracker: Using motion model: %s", motion_model_type_.c_str());
    }
    
    if (motion_model_type_ == "cv") {
        kalman_filter.predict(delta_t);
    } else if (motion_model_type_ == "ctrv") {
        kalman_filter.predictCTRV(delta_t);
    } else {  // "adaptive" or default
        kalman_filter.predictAdaptiveModel(delta_t);
    }

    // Check that object point cloud is not empty
    if (!object_cloud->points.empty())
    {
        // Define average point position
        PointType avg_pts_position;
        avg_pts_position.x = 0.0;
        avg_pts_position.y = 0.0;
        avg_pts_position.z = 0.0;

        // Define sum of weights for weighted average
        float weights_sum = 0.0;

        // Iterate over points in object point cloud
        for (std::size_t i = 0; i < object_cloud->size(); ++i)
        {
            // Retrieve point in object point cloud
            PointType pt = object_cloud->points[i];

            // Compute weight for each point based on timestamp stored in curvature
            float weight = exp(-gamma_ * (msg_timebase_ns - pt.curvature));

            // Update average point position with new point coordinates weighted
            avg_pts_position.x += pt.x * weight;
            avg_pts_position.y += pt.y * weight;
            avg_pts_position.z += pt.z * weight;

            // Sum weights
            weights_sum += weight;
        }

        // Compute average with safety check for division by zero
        if (weights_sum > 0.0) {
            avg_pts_position.x /= weights_sum;
            avg_pts_position.y /= weights_sum;
            avg_pts_position.z /= weights_sum;

            // Update observation with average of points detected in point cloud
            Eigen::VectorXd observation = Eigen::VectorXd::Zero(3);
            observation << avg_pts_position.x, avg_pts_position.y, avg_pts_position.z;
            kalman_filter.update(observation);
            
            ROS_DEBUG("AdaptiveTracker: Updated with %zu points", object_cloud->size());
        } else {
            ROS_WARN("AdaptiveTracker: Zero total weight in temporal weighting");
        }
    } else {
        // CRITICAL FIX: When no observations are available (object occluded),
        // we still continue with prediction-only mode
        ROS_DEBUG("AdaptiveTracker: No observations - continuing with prediction only");
        
        // The prediction has already been done above, so we're tracking through occlusion
        // The covariance will naturally grow, indicating increased uncertainty
    }
}

void AdaptiveTracker::computePose(geometry_msgs::PoseStamped& current_pose,
                                 geometry_msgs::PoseStamped& previous_pose,
                                 const Eigen::VectorXd& state)
{
    // Store the previous pose before updating current
    geometry_msgs::PoseStamped temp_previous = current_pose;
    
    // Define pose frame id and timestamp
    current_pose.header.frame_id = frame_id_;
    current_pose.header.stamp = ros::Time::now();

    // Compute position
    current_pose.pose.position.x = state[0];
    current_pose.pose.position.y = state[1];
    current_pose.pose.position.z = state[2];

    // Compute orientation
    // Check that we have a previous pose before computing orientation
    if (has_previous_pose_)
    {
        // Compute orientation angle (in radians) between current and last pose
        double theta = atan2((current_pose.pose.position.y - previous_pose.pose.position.y),
                            (current_pose.pose.position.x - previous_pose.pose.position.x));

        // Define orientation quaternion
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, theta); // Create quaternion from roll/pitch/yaw (in radians)
        geometry_msgs::Quaternion quaternion_msg;
        quaternion_msg = tf2::toMsg(orientation);

        current_pose.pose.orientation = quaternion_msg;
    }

    // Update previous pose for next iteration
    previous_pose = temp_previous;
    has_previous_pose_ = true;
}
