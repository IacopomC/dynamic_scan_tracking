// Include the ROS library
#include <ros/ros.h>

// Include ROS message types
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>

// Include the LIVOX LIDAR drivers library
#include <livox_ros_driver/CustomMsg.h>

// Include the PCL library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Include timing utilities
#include <chrono>

// Include components
#include "dynamic_scan_tracking/lidar_processor.h"
#include "dynamic_scan_tracking/adaptive_tracker.h"
#include "dynamic_scan_tracking/sensor_fusion.h"
#include "dynamic_scan_tracking/integration_time_manager.h"
#include "dynamic_scan_tracking/kalman_filter.h"
#include "dynamic_scan_tracking/performance_profiler.h"

typedef pcl::PointXYZINormal PointType;

class DynamicScanTrackingNode
{
private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscribers and Publishers
    ros::Subscriber sub_livox_lidar_;
    ros::Subscriber sub_initial_position_;
    ros::Publisher pub_object_pose_;
    ros::Publisher pub_object_velocity_;
    ros::Publisher pub_target_pcl_;
    ros::Publisher pub_raw_lidar_pcl_;  // Publisher for raw LiDAR point cloud

    // Components
    std::unique_ptr<LidarProcessor> lidar_processor_;
    std::unique_ptr<AdaptiveTracker> adaptive_tracker_;
    std::unique_ptr<SensorFusion> sensor_fusion_;
    std::unique_ptr<IntegrationTimeManager> integration_time_manager_;

    // Performance monitoring
    std::unique_ptr<PerformanceProfiler> profiler_;

    // Point clouds
    pcl::PointCloud<PointType>::Ptr point_cloud_current_scan_;
    pcl::PointCloud<PointType>::Ptr point_cloud_ast_;
    pcl::PointCloud<PointType>::Ptr point_cloud_adt_;
    pcl::PointCloud<PointType>::Ptr obj_cloud_ast_;
    pcl::PointCloud<PointType>::Ptr obj_cloud_adt_;

    // Deques for storing scans
    std::deque<pcl::PointCloud<PointType>> deque_ast_;
    std::deque<pcl::PointCloud<PointType>> deque_adt_;

    // Kalman filters
    KalmanFilter kf_ast_;
    KalmanFilter kf_adt_;

    // State tracking - simplified, no more growing vectors
    geometry_msgs::PoseStamped current_pose_ast_;
    geometry_msgs::PoseStamped current_pose_adt_;
    geometry_msgs::PoseStamped final_pose_;
    geometry_msgs::PoseStamped previous_pose_ast_;
    geometry_msgs::PoseStamped previous_pose_adt_;
    geometry_msgs::PoseStamped previous_pose_final_;

    // Future positions for tracking
    PointType future_position_ast_;
    PointType future_position_adt_;

    // Parameters loaded from YAML - better naming
    float search_radius_;
    float temporal_weight_decay_;           // Better name for gamma
    bool position_initialized_;
    bool drone_detection_enabled_;
    PointType initial_object_position_;     // Better name for obj_position
    double delta_t_pose_;

    // Integration time parameters from config
    int min_integration_time_ast_;
    int max_integration_time_ast_;
    int min_integration_time_adt_;
    int max_integration_time_adt_;
    float max_distance_ast_;
    float max_distance_adt_;

    // Integration time and distance tracking
    int optimal_integration_time_ast_;
    int optimal_integration_time_adt_;
    float distance_ast_;
    float distance_adt_;

    // Fusion results
    Eigen::VectorXd pose_fused_;
    Eigen::MatrixXd cov_fused_;
    double omega_;

    // Kalman filter initialization parameters with better names
    float initial_velocity_covariance_x_;   // Better name for sigma_p44
    float initial_velocity_covariance_y_;   // Better name for sigma_p55  
    float initial_velocity_covariance_z_;   // Better name for sigma_p66
    float measurement_noise_x_;             // Better name for sigma_lidar_x
    float measurement_noise_y_;             // Better name for sigma_lidar_y
    float measurement_noise_z_;             // Better name for sigma_lidar_z
    float process_noise_a_;                 // Acceleration noise for CTRV model
    float process_noise_yaw_accel_;         // Yaw acceleration noise for CTRV model
    float turn_rate_;                       // Turn rate for CTRV model
    
    // Adaptive model parameters
    bool use_adaptive_model_;               // Enable adaptive CV/CTRV model selection
    std::string motion_model_type_;         // Motion model type: "cv", "ctrv", or "adaptive"
    float turn_rate_threshold_;             // Minimum turn rate to consider
    float model_confidence_threshold_;      // Minimum confidence to switch to CTRV model

    // Topic and frame names from config
    std::string lidar_topic_;
    std::string init_position_topic_;
    std::string object_pose_topic_;
    std::string object_velocity_topic_;
    std::string target_pcl_topic_;
    std::string raw_lidar_pcl_topic_;   // Topic for raw LiDAR point cloud
    std::string lidar_frame_;
    
    // Visualization flags
    bool publish_raw_lidar_pcl_;        // Enable/disable raw LiDAR point cloud publishing

public:
    DynamicScanTrackingNode() : nh_(), pnh_("~")
    {
        initializeParameters();
        initializeComponents();
        initializePointClouds();
        initializeKalmanFilters();
        initializeROS();
        
        ROS_INFO("Dynamic Scan Tracking Node initialized successfully");
        ROS_INFO_STREAM("Namespace of public nh = " << nh_.getNamespace());
        ROS_INFO_STREAM("Namespace of private pnh = " << pnh_.getNamespace());
        ROS_INFO_STREAM("Drone detection enabled = " << drone_detection_enabled_);
        ROS_INFO_STREAM("Search radius = " << search_radius_);
        ROS_INFO_STREAM("Temporal weight decay = " << temporal_weight_decay_);
        ROS_INFO_STREAM("Initial position = (" << initial_object_position_.x << ", " 
                       << initial_object_position_.y << ", " << initial_object_position_.z << ")");
        ROS_INFO_STREAM("Performance profiling enabled = " << profiler_->isEnabled());
        ROS_INFO_STREAM("Raw LiDAR point cloud publishing enabled = " << publish_raw_lidar_pcl_);
        
        // Debug: Print loaded topic names to verify parameter loading
        ROS_INFO_STREAM("Loaded topics:");
        ROS_INFO_STREAM("  Input LiDAR: " << lidar_topic_);
        ROS_INFO_STREAM("  Output target PCL: " << target_pcl_topic_);
        ROS_INFO_STREAM("  Output raw LiDAR PCL: " << raw_lidar_pcl_topic_);
        ROS_INFO_STREAM("  LiDAR frame: " << lidar_frame_);
    }

private:
    void initializeParameters()
    {
        // Initialize default values
        position_initialized_ = false;
        delta_t_pose_ = 0.01;

        // Load tracking parameters from hierarchical YAML structure
        pnh_.param("tracking/search_radius", search_radius_, 0.2f);
        pnh_.param("tracking/temporal_weight_decay", temporal_weight_decay_, 0.000000005f);
        pnh_.param("tracking/delta_t_pose", delta_t_pose_, 0.01);
        
        // Load integration time parameters
        pnh_.param("integration_time/ast/min_integration_time", min_integration_time_ast_, 1);
        pnh_.param("integration_time/ast/max_integration_time", max_integration_time_ast_, 5);
        pnh_.param("integration_time/ast/max_distance", max_distance_ast_, 25.0f);
        
        pnh_.param("integration_time/adt/min_integration_time", min_integration_time_adt_, 10);
        pnh_.param("integration_time/adt/max_integration_time", max_integration_time_adt_, 50);
        pnh_.param("integration_time/adt/max_distance", max_distance_adt_, 60.0f);
        
        // Load Kalman filter parameters with better names
        pnh_.param("kalman_filter/initial_velocity_covariance/sigma_vx", initial_velocity_covariance_x_, 50.0f);
        pnh_.param("kalman_filter/initial_velocity_covariance/sigma_vy", initial_velocity_covariance_y_, 50.0f);
        pnh_.param("kalman_filter/initial_velocity_covariance/sigma_vz", initial_velocity_covariance_z_, 5.0f);
        
        pnh_.param("kalman_filter/measurement_noise/sigma_x", measurement_noise_x_, 0.1f);
        pnh_.param("kalman_filter/measurement_noise/sigma_y", measurement_noise_y_, 0.1f);
        pnh_.param("kalman_filter/measurement_noise/sigma_z", measurement_noise_z_, 0.1f);
        
        // Load process noise parameters - CRITICAL for real-world tracking performance
        pnh_.param("kalman_filter/process_noise/noise_a", process_noise_a_, 0.5f);
        pnh_.param("kalman_filter/process_noise/noise_yaw_accel", process_noise_yaw_accel_, 0.1f);
        
        // Load CTRV motion model parameters
        pnh_.param("kalman_filter/motion_model/turn_rate", turn_rate_, 0.1f);
        pnh_.param("kalman_filter/motion_model/type", motion_model_type_, std::string("adaptive"));
        pnh_.param("kalman_filter/motion_model/use_adaptive", use_adaptive_model_, true);
        pnh_.param("kalman_filter/motion_model/turn_rate_threshold", turn_rate_threshold_, 0.05f);
        pnh_.param("kalman_filter/motion_model/model_confidence_threshold", model_confidence_threshold_, 0.3f);

        // Load detection parameters
        pnh_.param("detection/drone_detection_enabled", drone_detection_enabled_, false);

        // Load topic names
        pnh_.param<std::string>("topics/input/lidar", lidar_topic_, "/livox/lidar");
        pnh_.param<std::string>("topics/input/init_position", init_position_topic_, "/init_position");
        pnh_.param<std::string>("topics/output/object_pose", object_pose_topic_, "/dynamic_scan_tracking/object_pose");
        pnh_.param<std::string>("topics/output/object_velocity", object_velocity_topic_, "/dynamic_scan_tracking/object_velocity");
        pnh_.param<std::string>("topics/output/target_pcl", target_pcl_topic_, "/target_pcl");
        pnh_.param<std::string>("topics/output/raw_lidar_pcl", raw_lidar_pcl_topic_, "/dynamic_scan_tracking/raw_lidar_pcl");
        
        // Load frame names
        pnh_.param<std::string>("frames/lidar_frame", lidar_frame_, "livox_frame");
        
        // Load visualization options
        pnh_.param("visualization/publish_raw_lidar_pcl", publish_raw_lidar_pcl_, true);

        // Initialize object position if drone detection is disabled
        if (!drone_detection_enabled_)
        {
            float init_x, init_y, init_z;
            pnh_.param("initial_position/x", init_x, 0.0f);
            pnh_.param("initial_position/y", init_y, 0.0f);
            pnh_.param("initial_position/z", init_z, 0.0f);
            
            initial_object_position_.x = init_x;
            initial_object_position_.y = init_y;
            initial_object_position_.z = init_z;
            position_initialized_ = true;
        }

        // Initialize optimal integration times
        optimal_integration_time_ast_ = max_integration_time_ast_;
        optimal_integration_time_adt_ = max_integration_time_adt_;
    }

    void initializeComponents()
    {
        // Load performance profiling settings
        bool enable_profiling;
        int profiling_window_size;
        pnh_.param("performance/enable_profiling", enable_profiling, true);
        pnh_.param("performance/profiling_window_size", profiling_window_size, 1000);
        
        // Initialize components with proper parameters
        lidar_processor_ = std::make_unique<LidarProcessor>();
        adaptive_tracker_ = std::make_unique<AdaptiveTracker>(search_radius_, temporal_weight_decay_, lidar_frame_);
        adaptive_tracker_->setMotionModelType(motion_model_type_);
        sensor_fusion_ = std::make_unique<SensorFusion>();
        integration_time_manager_ = std::make_unique<IntegrationTimeManager>(
            min_integration_time_ast_, max_integration_time_ast_,
            min_integration_time_adt_, max_integration_time_adt_,
            max_distance_ast_, max_distance_adt_);
        
        profiler_ = std::make_unique<PerformanceProfiler>(enable_profiling, profiling_window_size);
    }

    void initializePointClouds()
    {
        point_cloud_current_scan_.reset(new pcl::PointCloud<PointType>());
        point_cloud_ast_.reset(new pcl::PointCloud<PointType>());
        point_cloud_adt_.reset(new pcl::PointCloud<PointType>());
        obj_cloud_ast_.reset(new pcl::PointCloud<PointType>());
        obj_cloud_adt_.reset(new pcl::PointCloud<PointType>());
    }

    void initializeKalmanFilters()
    {
        // Initialize state vector
        Eigen::VectorXd x_in = Eigen::VectorXd::Zero(6);
        x_in << initial_object_position_.x, initial_object_position_.y, initial_object_position_.z, 0, 0, 0;

        // Initialize covariance matrix
        Eigen::MatrixXd P_in = Eigen::MatrixXd::Zero(6, 6);
        P_in(0, 0) = 1;
        P_in(1, 1) = 1;
        P_in(2, 2) = 1;
        P_in(3, 3) = initial_velocity_covariance_x_ * initial_velocity_covariance_x_;
        P_in(4, 4) = initial_velocity_covariance_y_ * initial_velocity_covariance_y_;
        P_in(5, 5) = initial_velocity_covariance_z_ * initial_velocity_covariance_z_;

        // Initialize transition matrix
        Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(6, 6);
        F_in(0, 3) = 1;
        F_in(1, 4) = 1;
        F_in(2, 5) = 1;

        // Initialize measurement matrix
        Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3, 6);
        H_in(0, 0) = 1;
        H_in(1, 1) = 1;
        H_in(2, 2) = 1;

        // Initialize measurement noise
        Eigen::MatrixXd R_in = Eigen::MatrixXd::Zero(3, 3);
        R_in(0, 0) = measurement_noise_x_ * measurement_noise_x_;
        R_in(1, 1) = measurement_noise_y_ * measurement_noise_y_;
        R_in(2, 2) = measurement_noise_z_ * measurement_noise_z_;

        // Initialize process noise
        Eigen::MatrixXd Q_in = Eigen::MatrixXd::Zero(6, 6);

        // Initialize both Kalman filters
        kf_ast_.init(x_in, P_in, F_in, H_in, R_in, Q_in);
        kf_adt_.init(x_in, P_in, F_in, H_in, R_in, Q_in);
        
        // Configure CTRV model parameters from config
        kf_ast_.setTurnRate(turn_rate_);
        kf_adt_.setTurnRate(turn_rate_);
        kf_ast_.setProcessNoise(process_noise_a_, process_noise_yaw_accel_);
        kf_adt_.setProcessNoise(process_noise_a_, process_noise_yaw_accel_);
        
        // Configure adaptive model selection
        bool use_adaptive = (motion_model_type_ == "adaptive");
        kf_ast_.setUseAdaptiveModel(use_adaptive);
        kf_adt_.setUseAdaptiveModel(use_adaptive);
        
        ROS_INFO("Motion model type: %s", motion_model_type_.c_str());

        // Initialize fusion matrices
        pose_fused_ = Eigen::VectorXd::Zero(6);
        cov_fused_ = Eigen::MatrixXd::Zero(6, 6);
    }

    void initializeROS()
    {
        // Subscribers using configurable topic names (limited queue size for real-time performance)
        sub_livox_lidar_ = nh_.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topic_, 5, &DynamicScanTrackingNode::livoxLidarCallback, this);
        sub_initial_position_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            init_position_topic_, 1, &DynamicScanTrackingNode::droneDetectionCallback, this);

        // Publishers using configurable topic names
        pub_object_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(object_pose_topic_, 1000);
        pub_object_velocity_ = nh_.advertise<geometry_msgs::TwistStamped>(object_velocity_topic_, 1000);
        pub_target_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>(target_pcl_topic_, 1000);
        pub_raw_lidar_pcl_ = nh_.advertise<sensor_msgs::PointCloud2>(raw_lidar_pcl_topic_, 1000);
    }

    void publishPointCloud(pcl::PointCloud<PointType>::Ptr& pcl_ptr, ros::Publisher& publisher)
    {
        PROFILE_SCOPE(*profiler_, "PublishPointCloud");
        
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*pcl_ptr.get(), pcl_ros_msg);
        pcl_ros_msg.header.frame_id = lidar_frame_;  // Use configurable frame name
        pcl_ros_msg.header.stamp = ros::Time::now();
        publisher.publish(pcl_ros_msg);
    }

    void droneDetectionCallback(const geometry_msgs::PoseStamped::ConstPtr& initial_position)
    {
        if (!position_initialized_ && drone_detection_enabled_)
        {
            Eigen::VectorXd x_in = Eigen::VectorXd::Zero(6);
            x_in << initial_position->pose.position.x, 
                    initial_position->pose.position.y, 
                    initial_position->pose.position.z, 0, 0, 0;
            
            // Reinitialize both Kalman filters with new position
            Eigen::MatrixXd P_in = Eigen::MatrixXd::Zero(6, 6);
            P_in(0, 0) = 1;
            P_in(1, 1) = 1;
            P_in(2, 2) = 1;
            P_in(3, 3) = initial_velocity_covariance_x_ * initial_velocity_covariance_x_;
            P_in(4, 4) = initial_velocity_covariance_y_ * initial_velocity_covariance_y_;
            P_in(5, 5) = initial_velocity_covariance_z_ * initial_velocity_covariance_z_;

            Eigen::MatrixXd F_in = Eigen::MatrixXd::Identity(6, 6);
            F_in(0, 3) = 1;
            F_in(1, 4) = 1;
            F_in(2, 5) = 1;

            Eigen::MatrixXd H_in = Eigen::MatrixXd::Zero(3, 6);
            H_in(0, 0) = 1;
            H_in(1, 1) = 1;
            H_in(2, 2) = 1;

            Eigen::MatrixXd R_in = Eigen::MatrixXd::Zero(3, 3);
            R_in(0, 0) = measurement_noise_x_ * measurement_noise_x_;
            R_in(1, 1) = measurement_noise_y_ * measurement_noise_y_;
            R_in(2, 2) = measurement_noise_z_ * measurement_noise_z_;

            Eigen::MatrixXd Q_in = Eigen::MatrixXd::Zero(6, 6);

            kf_ast_.init(x_in, P_in, F_in, H_in, R_in, Q_in);
            kf_adt_.init(x_in, P_in, F_in, H_in, R_in, Q_in);
            
            // Configure CTRV model parameters from config
            kf_ast_.setTurnRate(turn_rate_);
            kf_adt_.setTurnRate(turn_rate_);
            kf_ast_.setProcessNoise(process_noise_a_, process_noise_yaw_accel_);
            kf_adt_.setProcessNoise(process_noise_a_, process_noise_yaw_accel_);
            
            // Configure adaptive model selection
            bool use_adaptive = (motion_model_type_ == "adaptive");
            kf_ast_.setUseAdaptiveModel(use_adaptive);
            kf_adt_.setUseAdaptiveModel(use_adaptive);

            position_initialized_ = true;
        }
    }

    void livoxLidarCallback(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in)
    {
        PROFILE_SCOPE(*profiler_, "TotalProcessingTime");
        
        // Performance monitoring
        static auto last_callback_time = std::chrono::high_resolution_clock::now();
        auto current_time = std::chrono::high_resolution_clock::now();
        auto callback_interval = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_callback_time).count();
        last_callback_time = current_time;
        
        // Debug: Log callback reception and timing
        static int callback_count = 0;
        callback_count++;
        if (callback_count % 50 == 1) {  // Log every 50th callback
            ROS_INFO("Callback #%d: interval=%ldms, queue_delay=%.1fms, initialized=%s", 
                     callback_count, callback_interval,
                     (ros::Time::now() - livox_msg_in->header.stamp).toSec() * 1000.0,
                     position_initialized_ ? "true" : "false");
        }
        
        if (!position_initialized_) {
            if (callback_count % 100 == 1) {
                ROS_WARN("Position not initialized! Check initial_position config or enable drone_detection_enabled");
            }
            return;
        }

        unsigned long msg_timebase_ns;
        
        // Convert Livox message to PCL
        {
            PROFILE_SCOPE(*profiler_, "LivoxToPCL");
            if (!lidar_processor_->convertLivoxToPCL(livox_msg_in, point_cloud_current_scan_, msg_timebase_ns))
                return;
        }

        // Publish raw LiDAR point cloud for visualization (if enabled)
        if (publish_raw_lidar_pcl_) {
            publishPointCloud(point_cloud_current_scan_, pub_raw_lidar_pcl_);
        }

        // Track object with AST
        {
            PROFILE_SCOPE(*profiler_, "AdaptiveSparseTracking");
            trackObjectAdaptive(point_cloud_ast_, obj_cloud_ast_, deque_ast_, future_position_ast_,
                               kf_ast_, max_integration_time_ast_, optimal_integration_time_ast_,
                               current_pose_ast_, previous_pose_ast_, msg_timebase_ns);
        }

        // Track object with ADT
        {
            PROFILE_SCOPE(*profiler_, "AdaptiveDenseTracking");
            trackObjectAdaptive(point_cloud_adt_, obj_cloud_adt_, deque_adt_, future_position_adt_,
                               kf_adt_, max_integration_time_adt_, optimal_integration_time_adt_,
                               current_pose_adt_, previous_pose_adt_, msg_timebase_ns);
        }

        // Fuse estimates from both trackers
        {
            PROFILE_SCOPE(*profiler_, "SensorFusion");
            sensor_fusion_->fuseStatesICI(kf_ast_.x_, kf_ast_.P_, kf_adt_.x_, kf_adt_.P_,
                                                   pose_fused_, cov_fused_, omega_);
        }

        // Compute final pose
        {
            PROFILE_SCOPE(*profiler_, "ComputeFinalPose");
            adaptive_tracker_->computePose(final_pose_, previous_pose_final_, pose_fused_);
        }

        // Adjust integration times
        {
            PROFILE_SCOPE(*profiler_, "AdjustIntegrationTimes");
            integration_time_manager_->getOptimalIntegrationTimes(pose_fused_, 
                                                                 optimal_integration_time_ast_, 
                                                                 optimal_integration_time_adt_,
                                                                 distance_ast_, distance_adt_);
        }

        // Publish results
        {
            PROFILE_SCOPE(*profiler_, "PublishResults");
            publishResults();
        }

        // Visualize target point cloud
        publishPointCloud(obj_cloud_ast_, pub_target_pcl_);
    }

    void trackObjectAdaptive(pcl::PointCloud<PointType>::Ptr& integrated_cloud,
                            pcl::PointCloud<PointType>::Ptr& object_cloud,
                            std::deque<pcl::PointCloud<PointType>>& point_cloud_deque,
                            PointType& future_position,
                            KalmanFilter& kalman_filter,
                            int max_scans,
                            int optimal_integration_time,
                            geometry_msgs::PoseStamped& current_pose,
                            geometry_msgs::PoseStamped& previous_pose,
                            unsigned long msg_timebase_ns)
    {
        // Combine point clouds with adaptive integration time
        lidar_processor_->combinePointCloudsAdaptive(integrated_cloud, point_cloud_current_scan_,
                                                    max_scans, optimal_integration_time, point_cloud_deque);

        // Extract object cloud
        adaptive_tracker_->extractObjectCloud(integrated_cloud, object_cloud, future_position,
                                             kalman_filter, search_radius_);

        // Update position using Kalman filter
        adaptive_tracker_->updatePosition(object_cloud, kalman_filter, msg_timebase_ns, delta_t_pose_);

        // Compute pose
        adaptive_tracker_->computePose(current_pose, previous_pose, kalman_filter.x_);
    }

    void publishResults()
    {
        // Publish pose
        pub_object_pose_.publish(final_pose_);

        // Publish velocity
        geometry_msgs::TwistStamped current_velocity;
        current_velocity.header.frame_id = lidar_frame_;  // Use configurable frame name
        current_velocity.header.stamp = ros::Time::now();
        current_velocity.twist.linear.x = pose_fused_[3];
        current_velocity.twist.linear.y = pose_fused_[4];
        current_velocity.twist.linear.z = pose_fused_[5];
        pub_object_velocity_.publish(current_velocity);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic_scan_tracking_node");

    DynamicScanTrackingNode dynamic_scan_tracking_node;

    ros::MultiThreadedSpinner spinner(15);
    spinner.spin();

    return 0;
}
