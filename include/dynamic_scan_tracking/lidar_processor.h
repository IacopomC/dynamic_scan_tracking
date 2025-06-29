#ifndef LIDAR_PROCESSOR_H
#define LIDAR_PROCESSOR_H

#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <dynamic_scan_tracking/utils.h>

typedef pcl::PointXYZINormal PointType;

class LidarProcessor 
{
public:
    LidarProcessor();
    ~LidarProcessor() = default;

    /**
     * @brief Convert Livox message to PCL point cloud
     * @param livox_msg_in Input Livox message
     * @param point_cloud_out Output PCL point cloud
     * @param msg_timebase_ns Output message timebase in nanoseconds
     * @return true if conversion successful
     */
    bool convertLivoxToPCL(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in,
                          pcl::PointCloud<PointType>::Ptr& point_cloud_out,
                          unsigned long& msg_timebase_ns);

    /**
     * @brief Combine point clouds with adaptive integration time
     * @param current_cloud Current integrated cloud
     * @param new_msg_cloud New scan to add
     * @param max_scans Maximum number of scans to keep
     * @param optimal_integration_time Current optimal integration time
     * @param point_cloud_deque Deque storing recent scans
     */
    void combinePointCloudsAdaptive(pcl::PointCloud<PointType>::Ptr& current_cloud,
                                   pcl::PointCloud<PointType>::Ptr& new_msg_cloud,
                                   int max_scans,
                                   int optimal_integration_time,
                                   std::deque<pcl::PointCloud<PointType>>& point_cloud_deque);

private:
    std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data_;
};

#endif // LIDAR_PROCESSOR_H
