#include "dynamic_scan_tracking/lidar_processor.h"
#include <pcl_conversions/pcl_conversions.h>

LidarProcessor::LidarProcessor()
{
    // Constructor implementation
}

bool LidarProcessor::convertLivoxToPCL(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in,
                                      pcl::PointCloud<PointType>::Ptr& point_cloud_out,
                                      unsigned long& msg_timebase_ns)
{
    if (!point_cloud_out) {
        return false;
    }

    // Clear previous data
    point_cloud_out->clear();
    livox_data_.clear();

    // Convert Livox Message to PointCloud2
    livoxMsgToPCL(livox_msg_in, point_cloud_out, livox_data_);

    // Retrieve timestamp in nanoseconds from livox data
    if (!livox_data_.empty()) {
        msg_timebase_ns = livox_data_[0]->timebase;
        return true;
    }

    return false;
}

void LidarProcessor::combinePointCloudsAdaptive(pcl::PointCloud<PointType>::Ptr& current_cloud,
                                               pcl::PointCloud<PointType>::Ptr& new_msg_cloud,
                                               int max_scans,
                                               int optimal_integration_time,
                                               std::deque<pcl::PointCloud<PointType>>& point_cloud_deque)
{
    // Make a standalone copy of the new_msg_cloud object before storing it in the cloud
    // The deque contains the point cloud itself, not a pointer. Pointers end up being
    // updated to the same last PointCloud object
    pcl::PointCloud<PointType> new_cloud(*new_msg_cloud);

    // Add the new point cloud to the deque
    point_cloud_deque.push_front(new_cloud);

    // Clear the current point cloud
    current_cloud->clear();

    // Define number of scans to include in point cloud
    // Smallest value between optimal integration time and size of the deque
    int total_num_scans = (optimal_integration_time < point_cloud_deque.size()) ? 
                         optimal_integration_time : point_cloud_deque.size();

    // Add the points from the last total_num_scans point clouds to the current point cloud
    for (int i = 0; i < total_num_scans; ++i)
    {
        *current_cloud += point_cloud_deque[i];
    }

    // If the deque has more than max_scans elements, remove the oldest element
    if (point_cloud_deque.size() >= max_scans)
    {
        point_cloud_deque.pop_back();
    }
}
