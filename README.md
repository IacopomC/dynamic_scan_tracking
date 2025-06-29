<div align="center">
    <h1>UAV Tracking with Solid-State Lidars: Dynamic Multi-Frequency Scan Integration</h1>
    <a href="https://github.com/IacopomC/dynamic_scan_tracking/blob/main/LICENSE"><img src="https://img.shields.io/github/license/PRBonn/kiss-icp" /></a>
    <a href="https://github.com/IacopomC/dynamic_scan_tracking/blob/main"><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
    <a href="https://github.com/IacopomC/dynamic_scan_tracking/blob/main"><img src="https://img.shields.io/badge/Windows-0078D6?st&logo=windows&logoColor=white" /></a>
    <a href="https://github.com/IacopomC/dynamic_scan_tracking/blob/main"><img src="https://img.shields.io/badge/mac%20os-000000?&logo=apple&logoColor=white" /></a>
    <br />
    <br />
    <a href="https://tiers.github.io/dynamic_scan_tracking/">Project Page</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://arxiv.org/pdf/2304.12125.pdf">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/IacopomC/dynamic_scan_tracking/issues">Contact Us</a>
  <br />
  <br />
  <p align="center">
    <img src="doc/pipeline_diagram.png" width=99% />
  </p>

</div>

This is the code implementation for the paper [UAV Tracking with Solid-State Lidars: Dynamic Multi-Frequency Scan Integration](https://arxiv.org/abs/2304.12125).

<hr />

## Install
The code has been tested on Ubuntu 20.04 with ROS Noetic

### Dependencies
- PCL (Point Cloud Library)
- Eigen3
- Boost
- ROS Noetic
- Livox_ros_driver: Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver)

**Note**: The drone detection requires the point cloud to be in PointCloud2 format while the tracking uses the Livox CustomMsg data type. For the conversion use [this](https://github.com/koide3/livox_to_pointcloud2) repository and follow the instructions. If you use a different repository, change the value for the `self.lidar_sub` variable in the `livox_to_img.py` file accordingly.

### Build
```bash
cd ~/catkin_ws/src

git clone https://github.com/IacopomC/dynamic_scan_tracking

cd ~/catkin_ws

catkin build dynamic_scan_tracking

source devel/setup.bash
```

## Quick Start

### 1. Basic Tracking (Manual Initial Position)
```bash
# Terminal 1: Launch the tracking system
roslaunch dynamic_scan_tracking dynamic_scan_tracking.launch

# Terminal 2: Start your LiDAR driver (example for Livox)
roslaunch livox_ros_driver livox_lidar_msg.launch

# The system will start tracking from the configured initial position
```

## Configuration

### Motion Model Selection
Edit `config/dynamic_scan_tracking.yaml`:

```yaml
kalman_filter:
  motion_model:
    type: "adaptive"  # Options: "cv", "ctrv", "adaptive"
```

- **"adaptive"**: Automatically handles both straight flight and turns
- **"cv"**: Best for straight-line motion, fastest computation
- **"ctrv"**: Best for circular motion with known turn rate

### Key Parameters
- **Tracking accuracy**: Adjust `search_radius` (0.1-0.5m)
- **Motion agility**: Adjust `process_noise.noise_a` (1.0-5.0)
- **LiDAR noise**: Adjust `measurement_noise` values (0.05-0.2m)

See [TECHNICAL_DOCUMENTATION.md](/doc/TECHNICAL_DOCUMENTATION.md) for detailed parameter explanations.

## Monitoring

### ROS Topics
- `/dynamic_scan_tracking/object_pose`: Current target pose
- `/dynamic_scan_tracking/object_velocity`: Current target velocity  
- `/dynamic_scan_tracking/target_pcl`: Tracked point cloud (visualization)

## Documentation

- **[TECHNICAL_DOCUMENTATION.md](TECHNICAL_DOCUMENTATION.md)**: Comprehensive technical guide covering system architecture, motion models, configuration, and troubleshooting
- **[config/dynamic_scan_tracking.yaml](config/dynamic_scan_tracking.yaml)**: Configuration file with detailed parameter explanations

## Citation
If you use this code for any academic work, please cite the following publication:

```
@inproceedings{catalano2023uav,
  title={Uav tracking with solid-state lidars: dynamic multi-frequency scan integration},
  author={Catalano, Iacopo and Sier, Ha and Yu, Xianjia and Westerlund, Tomi and Queralta, Jorge Pena},
  booktitle={2023 21st International Conference on Advanced Robotics (ICAR)},
  pages={417--424},
  year={2023},
  organization={IEEE}
}
```
