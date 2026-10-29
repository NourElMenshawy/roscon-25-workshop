# Precision Landing Executor — ROS 2

A ROS 2 package implementing autonomous waypoint navigation and ArUco-guided precision landing using PX4 custom flight modes.

This package is based on the [Dronecode ROSCon 2025 Workshop](https://github.com/Dronecode/roscon-25-workshop) and has been extended for ROS 2 Lyrical, PX4 1.18, and asynchronous target processing using NVIDIA `stdexec`.

## Architecture

The application combines three main components:

- **CustomModeExecutor:** Manages the mission sequence, including takeoff, waypoint navigation, precision landing, and completion.
- **CustomWaypoints:** Navigates the drone through predefined local waypoints.
- **PrecisionLandCustom:** Uses ArUco observations to locate the landing target, approach it, and perform a controlled descent.

The precision-landing mode supports two processing backends:

- **`sync`:** Processes target observations synchronously.
- **`stdexec`:** Uses NVIDIA `stdexec` senders and receivers to schedule target processing asynchronously.

Both backends use the same precision-landing controller.

## Processing Pipeline

```text
PX4 CustomModeExecutor
         |
         v
       Takeoff
         |
         v
   CustomWaypoints
         |
         v
  PrecisionLandCustom
         |
         v
   ArUco /target_pose
         |
         v
  Target Processing
   (sync / stdexec)
         |
         v
 Search -> Approach -> Descend
         |
         v
   PX4 Landed State
```

The application transforms camera-relative marker observations into the PX4 local navigation frame and generates position or velocity setpoints for precision landing.

## Build

This package uses C++20, ROS 2 Lyrical, PX4 ROS 2 Interface Library, Eigen3, and NVIDIA `stdexec`.

From the configured ROS 2 workspace root, after making the required dependencies available:

```bash
colcon build --packages-up-to precision_land_executor --symlink-install
```

## Launch

Start the PX4 simulation using the `resources` package and the camera tracker using the `aruco_tracker` package.

After sourcing the workspace, start the precision-landing executor:

```bash
ros2 launch precision_land_executor precision_land_executor.launch.py
```

The default processing backend is `stdexec`. To select a backend explicitly:

```bash
ros2 launch precision_land_executor precision_land_executor.launch.py \
  processing_mode:=stdexec \
  processing_delay_ms:=0
```

For synchronous processing:

```bash
ros2 launch precision_land_executor precision_land_executor.launch.py \
  processing_mode:=sync \
  processing_delay_ms:=0
```

The `processing_delay_ms` parameter introduces artificial processing delay for comparing execution behavior under load.

The executor registers the custom flight modes with PX4. The mission is activated separately through the PX4 mode interface.

## Performance Comparison

The application provides benchmark logging for both processing backends, including callback execution time, pipeline latency, queue waiting, control-loop timing, and processed or dropped observations.

These measurements support comparisons between synchronous and asynchronous processing under identical simulation conditions.

## Original Work and Attribution

This package is derived from the precision-landing example provided by the Dronecode ROSCon 2025 PX4/ROS 2 workshop.

**Original repository:** https://github.com/Dronecode/roscon-25-workshop

**Original documentation:** [README.upstream.md](README.upstream.md)

The original project's attribution and applicable licensing requirements are preserved.

This package is an independent experimental extension and is not an official Dronecode workshop release.
