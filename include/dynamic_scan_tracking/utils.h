// function taken from https://github.com/Livox-SDK/livox_mapping/blob/master/src/livox_repub.cpp

#ifndef UTILS
#define UTILS

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#include <iostream>

uint64_t TO_MERGE_CNT = 1;

using Eigen::MatrixXd;
using Eigen::VectorXd;

inline void livoxMsgToPCL(const livox_ros_driver::CustomMsgConstPtr& livox_msg_ptr,
                          pcl::PointCloud<pcl::PointXYZINormal>::Ptr& pcl_ptr,
                          std::vector<livox_ros_driver::CustomMsgConstPtr>& livox_data)
{

    livox_data.push_back(livox_msg_ptr);
    if (livox_data.size() < TO_MERGE_CNT) return;

    for (size_t j = 0; j < livox_data.size(); j++) {
        auto& livox_msg = livox_data[j];
        auto time_end = livox_msg->points.back().offset_time;
        for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
            pcl::PointXYZINormal pt;
            pt.x = livox_msg->points[i].x;
            pt.y = livox_msg->points[i].y;
            pt.z = livox_msg->points[i].z;
            // float s = livox_msg->points[i].offset_time / (float)time_end;

            pt.intensity = livox_msg->points[i].line + livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
            // pt.curvature = s*0.1;
            pt.curvature = livox_msg->timebase; // save timestamp last point into curvature to compute weights for pose calculation
            pcl_ptr->push_back(pt);
        }
    }
}

#endif // UTILS

