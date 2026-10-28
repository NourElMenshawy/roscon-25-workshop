# Precision Landing Executor — PX4 ROS 2 Workshop Adaptation

**Status (2026-10-15):** Builds and has completed a takeoff → waypoint → ArUco precision-landing mission in the native ROS 2 Lyrical/PX4 SITL environment, using the `resources` Gazebo launcher. The sender/receiver (`std::execution`) extension is **not yet implemented**. Lifecycle and timestamp hardening are documented below.

Adapted from the [Dronecode ROSCon 2025 `precision_land_executor` example](https://github.com/Dronecode/roscon-25-workshop/tree/main/px4_roscon_workshop/precision_land_executor). This document describes the **local adaptation**, including namespaced PX4 topics and the `aruco` world, and captures the debugging results up to the working baseline.

## What the demo does

An autonomous mission takes off, follows a list of local NED waypoints, activates an ArUco-guided landing mode, aligns above the marker, descends, and observes PX4's landed indication.

```text
QGroundControl / mode activation
               │
               ▼
CustomModeExecutor (mission-level state machine)
   Takeoff (2 m) → CustomWaypoints → PrecisionLand → WaitUntilDisarmed
                       │                  │
                       ▼                  ▼
            Position setpoints       Search → Approach → Descend → Finished
                                              ▲        │
                                              │        ▼
Gazebo downward camera → aruco_tracker → /target_pose  PX4 trajectory setpoint
PX4 odometry / attitude ───────────────────────┘        │
                                                       ▼
                                                   PX4 control
```

### Three levels of execution

- **PX4 `ModeExecutorBase`:** requests takeoff and sequences the *flight modes*.
- **ROS 2 executor:** runs callbacks (target observations, PX4 landing events, mode updates).
- **C++26 `std::execution`:** proposed *computation scheduler* for transform/freshness work. This will **not** replace the PX4 mission executor or the flight controller.

## Package files and ownership

| File | Role |
| --- | --- |
| `CustomMode_v2.hpp` | Declares `CustomWaypoints`, `PrecisionLand`, `ArucoTag`, mode state, configuration and ROS handles. |
| `CustomMode_v2.cpp` | Implements waypoint navigation, target callback/transform, landing state machine, PI controller and search spiral. |
| `CustomModeExecutor_v2.hpp` | Declares the mission executor and its states. |
| `CustomModeExecutor_v2.cpp` | Implements `takeoff()`, `scheduleMode()`, `waitUntilDisarmed()`, and creates the combined node. |
| `launch/precision_land_executor.launch.py` | Starts the executor node; local adaptation uses `namespace="px4_0"` and `use_sim_time=True`. |
| `cfg/` | Workshop configuration assets where present. |
| `CMakeLists.txt`, `package.xml` | Executable build, public dependency linking and ROS package metadata. |

The application combines `CustomModeExecutor`, `CustomWaypoints`, and `PrecisionLand` in a single `px4_ros2::NodeWithModeExecutor<...>` and spins the ROS node. The two mode objects persist across activations.

## Dependencies and runtime topics

Required software components:

- `px4_ros2_cpp` (used with a local `release/1.17` checkout during the native build)
- A firmware-compatible `px4_msgs` checkout
- ROS 2 Lyrical, `rclcpp`, Eigen3, `geometry_msgs`, `tf2`, `tf2_ros`
- PX4 SITL + Gazebo + Micro XRCE-DDS Agent
- A working ROS camera/image-info bridge and ArUco tracker

Observed ROS topics:

| Topic | Type / reason |
| --- | --- |
| `/target_pose` | `geometry_msgs/msg/PoseStamped`; ArUco target pose relative to the camera. |
| `/camera_info` | `sensor_msgs/msg/CameraInfo`; camera intrinsics needed for pose estimation. |
| `/px4_0/fmu/out/vehicle_odometry` | PX4 odometry feed (also accessed via PX4 interface wrappers). |
| `/px4_0/fmu/out/vehicle_status_v1` | Versioned vehicle status in this simulation. |
| `/px4_0/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected`; `landed` condition. |
| `/px4_0/fmu/in/register_ext_component_request` | PX4 external-component registration requests. |
| `/px4_0/fmu/out/register_ext_component_reply` | PX4 external-component registration replies. |
| `/clock` | Simulation time (must advance when `use_sim_time` is enabled). |

The local source uses a best-effort depth-1 subscription to `/target_pose`. `Node(namespace="px4_0")` is needed for the workshop interface to discover the namespaced FMU topics in this configuration. Absolute topic names in your own C++ subscriptions do **not** inherit the node namespace: the landing detection subscription was changed to `/px4_0/fmu/out/vehicle_land_detected`.

## Mission-level state machine — `CustomModeExecutor`

The combined node registers both `CustomWaypoints` and `PrecisionLandCustom` with the PX4 external-mode interface.

1. **Takeoff:** `takeoff(..., 2.0f)` requests takeoff and reports completion to the executor.
2. **CustomWaypoints:** `scheduleMode(ownedMode().id(), ...)` activates waypoint navigation.
3. **PrecisionLand:** `scheduleMode(_second_mode.id(), ...)` activates the marker-guided mode when the waypoint mode reports completion.
4. **WaitUntilDisarmed:** `waitUntilDisarmed(...)` waits until PX4 disarms. It does not itself command an additional landing maneuver.

The original executor logs an unsuccessful prior result and still transitions onward. This is workshop-example behavior; an operational mission should define explicit failure/failsafe policy before continuing.

### Waypoints and their coordinate frame

`CustomWaypoints::onActivate()` fills a vector of local **PX4 NED** setpoints. NED axes are X north, Y east, Z down; a negative Z means altitude above the local origin. The original list was:

| # | PX4 local NED waypoint (m) |
| --- | --- |
| 1 | `(1, 2.5, -3)` |
| 2 | `(14, 2.5, -3)` |
| 3 | `(14, 8, -3)` |
| 4 | `(-4, 8, -3)` |

The original workshop spawned its tag in the Gazebo `walls` world at **ENU `(8, -4, 0)`**, corresponding to local NED horizontal coordinates `(-4, 8)` when the world/local frames are aligned.

Our native setup uses **`world:=aruco`** and an ArUco tag near the world origin; the last waypoint was moved to approximately **NED `(0, 0, -3)`** to align with it. This depends on the configured spawn offsets and frame origins—verify the actual marker pose if the setup changes.

Waypoints advance when position is within **0.5 m** of the setpoint. When the vector is exhausted, the waypoint mode reports `px4_ros2::Result::Success`.

## PrecisionLand mode — control-level state machine

| State | Setpoint/action | Transition |
| --- | --- | --- |
| `Idle` | No action | Entered after error; otherwise inactive. |
| `Search` | Spiral **position** setpoints around the current drone location | Target appears valid to the Search check. |
| `Approach` | **Position** setpoint at marker XY and current altitude | Position error < 0.25 m **and** speed < 0.25 m/s. |
| `Descend` | **Velocity** setpoint: PI horizontal correction and positive-NED-Z descent | `VehicleLandDetected.landed == true`. |
| `Finished` | Reports `px4_ros2::Result::Success` | Mode executor observes completion. |

The current `Search` code initially checks only for non-NaN `_tag.position.x()`; the more complete `checkTargetTimeout()` checks a valid timestamp and **3-second** age limit. The inconsistency is a known reactivation bug, documented below.

### Search spiral

On activation, `generateSearchWaypoints()` creates outward/inward spiral points around the **current vehicle position**, with approximately:

- Maximum radius **1 m**
- `points_per_layer = 16` (17 sampled points per outward leg)
- NED Z changed by **0.5 m** between outward and inward traversals

It is **not** a global search for a distant marker. The last waypoint must put the drone close enough for the downward camera to observe the marker.

### ArUco pose and coordinate transformation

The tracker uses camera images, intrinsic calibration (`CameraInfo`), known marker geometry, and ArUco/PnP pose estimation to publish `/target_pose`.

The mode transforms marker pose from camera coordinates into PX4 local NED using:

```text
world_T_tag = world_T_drone × drone_T_camera × camera_T_tag
```

In mathematical notation:

`T_WT = T_WB · T_BC · T_CT`

`getTagWorld()` uses PX4 estimated vehicle position, vehicle attitude, a fixed camera rotation and zero camera translation, and the received marker translation/orientation. The transform is **hardcoded rather than queried from a TF tree**. Confirm that its fixed camera mounting model matches the simulated camera if changing drones.

The target callback currently assigns `timestamp = _node.now()` when it processes the incoming message. It does **not** preserve `PoseStamped.header.stamp` as the control target timestamp, so it cannot yet measure full camera-to-controller observation age. PX4 state is read when processing the pose, which may be later than image capture.

### Descending PI controller

Horizontal position error is defined as `e = position_drone - position_tag` in local NED.

```text
vx = clamp(-Kp * ex - Ki * integral_x, -max_velocity, +max_velocity)
vy = clamp(-Kp * ey - Ki * integral_y, -max_velocity, +max_velocity)
vz = +descent_vel   # positive NED Z means DOWN
```

Each integral accumulator is clamped to `[-max_velocity, +max_velocity]`. The code does not multiply the accumulated error by `dt_s`; the integral therefore depends on update frequency. Each XY component is individually clamped (not the combined vector norm). The yaw setpoint is extracted from the target orientation.

Default declared parameters (verify local YAML/overrides):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `descent_vel` | `1.0` | Desired downward speed in m/s (positive NED Z). |
| `vel_p_gain` | `1.5` | Horizontal proportional gain. |
| `vel_i_gain` | `0.0` | Horizontal integral gain (disabled by default). |
| `max_velocity` | `3.0` | Per-axis XY velocity clamp (m/s). |
| `target_timeout` | `3.0` | Target timeout (seconds). |
| `delta_position` | `0.25` | Position tolerance for Approach completion (metres). |
| `delta_velocity` | `0.25` | Speed tolerance for Approach completion (m/s). |

The condition for `Approach → Descend` uses both position and velocity norms. `positionReached()` explicitly notes that the implementation does **not** handle a moving target.

## Build and run

```bash
# Source the underlay and any preceding workspaces.
source /opt/ros/lyrical/setup.zsh
cd ~/workspace/px4_ros_wc

colcon build --symlink-install \
  --packages-up-to precision_land_executor \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source install/local_setup.zsh
```

Terminal 1, with the workspace containing `resources` sourced:

```bash
ros2 launch resources simulation.launch.py \
  world:=aruco model:=x500_mono_cam_down_aruco
```

Terminal 2, after starting the camera tracker if not included in the simulation launch:

```bash
ros2 topic hz /camera_info
ros2 topic echo /px4_0/fmu/out/vehicle_status_v1 --once
# Target appears when the camera sees the marker:
ros2 topic hz /target_pose
```

Terminal 3:

```bash
ros2 launch precision_land_executor precision_land_executor.launch.py
```

Then use **QGroundControl** to select and activate the **CustomWaypoints** mode/executor, and let it perform takeoff → navigation → precision landing. Starting the node registers the modes; registration alone does **not** activate a mission. Do not test a manually activated landing immediately before a full executor mission without restarting or fixing the reactivation state bug.

### Successful runtime indications

```text
Checking message compatibility...
Registering 'CustomWaypoints' ...
Got RegisterExtComponentReply
Registering 'PrecisionLandCustom' ...
Got RegisterExtComponentReply
CustomModeExecutor activated
Initiating takeoff...
CustomWaypoints mode activated
All waypoints completed.
Mode 'PrecisionLandCustom' activated
Switching to Search
Target acquired
Switching to Approach
Switching to Descend
Switching to Finished
```

Exact output can differ; visually verify touchdown and use `/px4_0/fmu/out/vehicle_land_detected` to confirm landed state rather than relying only on the last log line.

## Known issues and hardening before stdexec

1. **Stale target on reactivation (observed):** A direct manual landing populated `_tag`; later the same persistent `PrecisionLand` object was activated by the waypoint executor. `onActivate()` did not clear `_tag`, so `Search` immediately entered `Approach` using an old target; `checkTargetTimeout()` rejected it about 20 ms later. Reset target/timestamp, search index, integrals, and transient flags on each activation, and use a **freshness check** for `Search → Approach`.
2. **Repeated completion (observed):** `CustomWaypoints` may log completion multiple times before deactivation; `PrecisionLand::Finished` can call `completed()` again. Add one-shot completion guards reset on activation.
3. **Timestamp correctness:** The incoming `PoseStamped.header.stamp` should reflect acquisition time (verify the tracker preserves image capture timestamp). Distinguish **source age**, **callback processing time**, and **age at controller use**; ensure all time values use consistent clocks.
4. **State and thread safety:** `_tag` and other mode state are mutated from callbacks and read in `updateSetpoint()`. Do not write these from a new worker without a synchronized handoff. Capture odometry/attitude into an immutable snapshot *before* asynchronous computation. Avoid mixing camera state from one timestamp with PX4 state from another.
5. **Mode lifecycle:** Clear or invalidate target-related state during deactivation; ensure no asynchronous result from an earlier activation can update the current controller. A generation/activation identifier is useful for rejecting late completions.
6. **Marker/world geometry:** The original `walls` tag placement and `aruco` world's tag placement differ. Check actual marker pose and NED/ENU origin alignment.
7. **Fixed camera extrinsics / dynamic pad:** The camera transform assumes zero offset with a fixed rotation; and the controller is for a static marker, not a fully modeled moving target.
8. **Connection timeout after mission:** A later run logged `Timeout, no request received from FMU` after deactivation. The evidence does **not** establish whether PX4 rebooted, agent communication stopped, or a mode-lifecycle interaction was involved. Debug this separately if reproducible.

## Planned C++26 `std::execution` demonstration

**Talk title:** *From Callbacks to Senders: Rethinking PX4 Precision Landing with C++26*.

Goal: test whether making the perception-transform pipeline asynchronous and **freshness-aware** improves the age of data used by the controller under artificial load. A sender does **not** make pose math inherently faster and does **not** automatically interrupt arbitrary work.

### Baseline (not yet refactored)

```text
/target_pose callback
  → read current PX4 state
  → getTagWorld() transform
  → overwrite _tag
  → controller uses _tag
```

### Proposed sender pipeline (not yet implemented)

```text
/target_pose callback
  → immutable PoseSnapshot(tag pose + source time + matched PX4 state)
  → schedule on bounded worker pool
  → then(transform to NED)
  → then(validate freshness / generation)
  → synchronized delivery to controller
  → controller publishes setpoints at its existing cadence
```

Use [NVIDIA/stdexec](https://github.com/NVIDIA/stdexec) initially, retaining the original mode executor and PX4 position/velocity controllers. Manage sender-operation lifetimes explicitly; a stopped signal only cancels work where operations cooperate. Use bounded queues or a latest-frame policy to avoid producing correct commands from old images.

### Comparison methodology

- Run the **same** world, model, controller gains, camera rate and route.
- Compare the synchronous baseline against a scheduled, bounded/freshest-data pipeline.
- Add controlled delays (for example **0 ms, 30 ms and 100 ms**) to target processing.
- Record frame/capture timestamp, callback receipt timestamp, controller update timestamp, processing duration, observation age at control, dropped/stopped observations, and landing/touchdown accuracy.
- Report latency distribution (median, p95, max) and behavior on target loss; do not claim improvement before collecting real measurements.

**Immediate next coding step:** preserve the working baseline in Git, reset/validate mode state per activation, extract a pure `getTagWorld(snapshot)` transform, instrument source-to-control age, rerun the flight, then add `stdexec` as a separate implementation.

## Attribution and references

- [Dronecode ROSCon 2025 workshop](https://github.com/Dronecode/roscon-25-workshop)
- [Workshop `precision_land_executor`](https://github.com/Dronecode/roscon-25-workshop/tree/main/px4_roscon_workshop/precision_land_executor)
- [PX4 ROS 2 Interface Library](https://github.com/Auterion/px4-ros2-interface-lib)
- [NVIDIA/stdexec](https://github.com/NVIDIA/stdexec)

Retain the original upstream license and attribution in any redistributed source files.
