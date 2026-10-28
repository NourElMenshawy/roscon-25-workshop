# ArUco Tracker — ROS 2

A ROS 2 package for detecting ArUco markers from camera images and publishing their estimated poses for use in PX4 precision-landing applications.

This package is based on the [Dronecode ROSCon 2025 Workshop](https://github.com/Dronecode/roscon-25-workshop) and has been refactored as part of the PX4 precision-landing and C++26 `std::execution` demonstrations.

## Architecture

The original `ArucoTracker` implementation has been reorganized to separate ArUco detection from ROS 2 integration.

### `aruco_detector.hpp`

Contains the detection-related interface, responsible for the ArUco marker processing used by the tracker.

### `aruco_tracker.hpp`

Defines the ROS 2 tracking interface, responsible for integrating camera observations with the detection functionality and publishing the resulting marker pose.

### `src/ArucoTracker.cpp`

Contains the implementation built into the `aruco_tracker` executable.

This separation is intended to make the detection functionality easier to reuse in future applications, including multi-drone perception and asynchronous processing experiments.

## Processing Pipeline

```text
Gazebo Camera
      |
      v
ROS 2 Camera Image
      |
      v
ArUco Detection
      |
      v
Marker Pose Estimation
      |
      v
/target_pose (PoseStamped)
      |
      v
PX4 Precision Landing
```

The published marker pose is expressed relative to the camera coordinate frame.

Conversion into the PX4 local navigation frame is performed by the downstream precision-landing application, not by the ArUco tracker.

## Build

This package uses C++20, ROS 2, and OpenCV with the ArUco module.

From the ROS 2 workspace root:

```bash
colcon build --packages-select aruco_tracker --symlink-install
```

## Launch

After sourcing the workspace:

```bash
ros2 launch aruco_tracker aruco_tracker.launch.py
```

Camera topics and detection parameters are configured through the package's launch and configuration files.

## Relationship to the Precision-Landing Demonstration

The ArUco tracker provides camera-relative target observations to the `precision_land_executor` package.

The precision-landing application processes these observations to calculate the target position in the drone's local navigation frame.

The asynchronous execution experiments using NVIDIA `stdexec` are implemented in the precision-landing application, not in this tracker package.

## Original Work and Attribution

This package is derived from the ArUco tracker provided by the Dronecode ROSCon 2025 PX4/ROS 2 workshop.

**Original repository:** https://github.com/Dronecode/roscon-25-workshop

**Original documentation:** [README.upstream.md](README.upstream.md)

The modified package retains the original project's attribution and applicable licensing requirements.

The refactoring, integration adjustments, and subsequent experimental extensions are documented as changes relative to the workshop implementation.

This package is part of an independent experimental extension and is not an official Dronecode workshop release.
