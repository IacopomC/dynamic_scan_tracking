# Dynamic LiDAR Tracking System - Technical Documentation

## Overview

This system implements a robust real-time UAV tracking solution using solid-state LiDAR sensors. The core innovation is the use of **dual parallel trackers** with different point cloud integration times, combined with **adaptive motion models** that automatically switch between Constant Velocity (CV) and Constant Turn Rate and Velocity (CTRV) models based on detected motion patterns.

## System Architecture

### Core Components

1. **LidarProcessor**: Handles point cloud preprocessing and segmentation
2. **AdaptiveTracker**: Implements the tracking algorithm with configurable motion models
3. **SensorFusion**: Fuses estimates from dual trackers using Intersection Covariance Intersection (ICI)
4. **IntegrationTimeManager**: Manages adaptive integration times based on target distance
5. **KalmanFilter**: Implements CV, CTRV, and adaptive motion models
6. **PerformanceProfiler**: Monitors system performance and timing

### Dual Tracker Strategy

The system runs two parallel trackers with complementary characteristics:

- **AST (Adaptive Sparse Tracking)**: 1-5 scans integration, optimized for close objects (< 25m)
- **ADT (Adaptive Dense Tracking)**: 10-50 scans integration, optimized for distant objects (< 60m)

This approach provides both **accuracy** (dense tracking for distant targets) and **robustness** (sparse tracking for rapid motion).

## Motion Models

### 1. Constant Velocity (CV) Model
```
State: [x, y, z, vx, vy, vz]
Prediction: x_k+1 = x_k + vx * dt
```
- **Best for**: Straight-line motion, gentle maneuvers
- **Advantages**: Simple, stable, no parameter tuning
- **Limitations**: Cannot handle sharp turns

### 2. Constant Turn Rate and Velocity (CTRV) Model
```
State: [x, y, z, vx, vy, vz]
Prediction: Non-linear based on turn rate and speed
```
- **Best for**: Circular motion, banked turns
- **Advantages**: Handles turning motion accurately
- **Limitations**: Requires known/estimated turn rate, poor for straight motion

### 3. Adaptive Model
Automatically blends CV and CTRV predictions based on:
- **Turn rate estimation** from velocity history analysis
- **Motion consistency confidence** assessment
- **Smooth model transitions** to avoid tracking discontinuities

**Algorithm**:
1. Analyze recent velocity vector changes
2. Estimate current turn rate and confidence
3. Run both CV and CTRV predictions in parallel
4. Blend results based on confidence: CV for straight motion, CTRV for turns

## Configuration

### Motion Model Selection
Set in `config/dynamic_scan_tracking.yaml`:

```yaml
kalman_filter:
  motion_model:
    type: "adaptive"  # Options: "cv", "ctrv", "adaptive"
    turn_rate: 0.1    # Fixed turn rate for CTRV model (rad/s)
```

### Key Parameters

#### Tracking Parameters
- `search_radius`: KDTree search radius for point association (0.2m recommended)
- `temporal_weight_decay`: Exponential decay for temporal weighting (5e-9 recommended)

#### Integration Times
- AST: 1-5 scans for close objects (< 25m)
- ADT: 10-50 scans for distant objects (< 60m)

**Adaptive Integration**: The system automatically adjusts integration times based on target distance:
- **Close targets**: Use fewer scans (faster response, less latency)
- **Distant targets**: Use more scans (better noise reduction, more stability)
- **Formula**: `integration_time = min_time + (max_time - min_time) * (distance / max_distance)`

#### Kalman Filter Tuning
- `process_noise.noise_a`: Acceleration uncertainty (2.0 for agile UAVs)
- `process_noise.noise_yaw_accel`: Turn rate change uncertainty (0.5 recommended)
- `measurement_noise`: LiDAR accuracy (0.1m typical)

## Performance Characteristics

### Computational Complexity
- **Real-time capable**: ~10-20ms per frame on modern CPUs
- **Memory efficient**: Pre-allocated objects, minimal dynamic allocation

### Tracking Robustness
- **Occlusion handling**: Continues tracking through prediction when measurements unavailable
- **Motion adaptability**: Handles both straight flight and sharp turns automatically
- **Noise resilience**: Kalman filtering provides inherent noise rejection

## Usage Scenarios

### Recommended Model Selection

1. **"adaptive"** (Default): Best overall performance
   - Handles all motion patterns automatically
   - No parameter tuning required
   - Slightly higher computational cost

2. **"cv"**: For predictable straight-line motion
   - Lowest computational cost
   - Most stable for linear trajectories
   - Poor performance during turns

3. **"ctrv"**: For known circular motion patterns
   - Best for coordinated turns with known turn rate
   - Requires accurate turn rate parameter
   - Poor for straight motion or varying turn rates

### Tuning Guidelines

1. **For fast, agile UAVs**: Increase `noise_a` (3.0-5.0)
2. **For smooth flight patterns**: Decrease `noise_a` (1.0-2.0)
3. **For noisy LiDAR**: Increase `measurement_noise` values

## Implementation Details

### Thread Safety
- Each tracker instance operates independently
- Sensor fusion combines results thread-safely
- Configuration updates require node restart

### Numerical Stability
- Joseph form covariance updates for positive definiteness
- LLT decomposition for matrix inversions
- Bounded confidence values to prevent numerical issues

### Memory Management
- Pre-allocated Eigen matrices for hot paths
- Smart pointers for component lifecycle management
- Minimal dynamic allocation in tracking loops

## References

- [Original Paper](https://arxiv.org/abs/2304.12125): UAV Tracking with Solid-State Lidars
- [Intersection Covariance Intersection](https://ieeexplore.ieee.org/document/10.1109/AES.2000.880249): Sensor fusion algorithm
