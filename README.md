# ekf2_fusion_pkg

## Overview 
This project is a ROS2 node that performs sensor fusion between IMU and image-based GPS data.

- It subscribes to IMU velocity data, converts it into position (X, Y), and transforms it into latitude and longitude coordinates so that it matches the GPS data format.
- It also calls a service that returns visual GPS coordinates (from camera image).
- Using a weighted fusion technique based on covariance, it combines both sensors to get a more accurate position.
- The final fused position is published to a ROS2 topic.

## What This Node Does (Fusion Logic & Features)

This node integrates two asynchronous sensor sources to output a smoother and more reliable GPS estimate. Here's how:

- **IMU Data Handling**: Subscribes to `/filtered/imu/data`, extracts velocity (in meters), integrates it to estimate position, and then converts it to lat/lon using `pyproj.Geod`.

- **Image-Based GPS Estimate**: Calls the `/image_pose` service to get visual GPS (lat/lon) estimates using camera-based localization.

- **Covariance Estimation**: Each data source has an associated uncertainty (covariance). Lower covariance = more accurate = more trust.

- **Weighted Fusion**: Combines IMU and visual GPS using weighted average:
  
  \[
  \text{fused\_pos} = \frac{\text{pos}_\text{imu}/\text{cov}_\text{imu} + \text{pos}_\text{vis}/\text{cov}_\text{vis}}{1/\text{cov}_\text{imu} + 1/\text{cov}_\text{vis}}
  \]
  
  This ensures that sensors with higher certainty contribute more.

- **Result Publishing**: The final fused GPS is published on `/fused_pose`.

## ROS2 Integration

- Package name: `ekf2_fusion_pkg`
- Main file: `ekf_fusion.py`
- Build using: `colcon build`
- Run: 
  `bash`
  `ros2 run ekf2_fusion_pkg ekf_fusion`

## Topics Used

### Subscribed Topic:
- `/filtered/imu/data` IMU velocity (TwistStamped)

### Service Called:
- `/image_pose` (ImagePose service) → Gets GPS position estimate from image

### Published Topic:
- `/fused_pose` Final fused GPS-IMU position (PoseStamped)

## Assumptions

- Z-axis is ignored (only X and Y used).
- GPS position from camera is trusted when available.
- Orientation filters out invalid data (e.g. camera not facing ground).

## Future Improvements

- Add full EKF2 filter for better prediction

### Thank you :)
