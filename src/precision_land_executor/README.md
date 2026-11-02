# Precision Landing Executor — Composable PX4 Landing Missions

A ROS 2 package that combines **autonomous waypoint navigation**, **ArUco-guided
precision landing**, and **secondary position landing** through PX4 custom flight
modes. In the two-drone simulation, the package also evaluates follower landing
strategies using NVIDIA `stdexec` senders and receivers.

The package builds on the [Dronecode ROSCon 2025 PX4/ROS 2 workshop](https://github.com/Dronecode/roscon-25-workshop)
and is adapted for ROS 2 Lyrical, PX4 1.18, and a coordinated two-drone landing
mission. It uses C++20 with NVIDIA `stdexec`'s sender/receiver implementation.

## Architecture

Each drone runs the same PX4 mode-executor application:

```text
            precision_land_executor (one instance per PX4 vehicle)
                                  |
             px4_ros2::NodeWithModeExecutor<...>
                                  |
                 +----------------+----------------+
                 |                                 |
          CustomWaypoints                  PrecisionLandCustom
          - waypoint approach             - ArUco search
          - secondary-pad route           - marker alignment
          - controlled descent            - precision descent
                 |                                 |
                 +----------------+----------------+
                                  |
                         CustomModeExecutor
                      - owns PX4 mode handoffs
                      - reacts to mission status
                      - confirms landed/disarmed
                      - evaluates follower strategy
                                  |
                         NVIDIA stdexec
               schedule / when_all / then / recovery
```

The **landing coordinator** arbitrates exclusive access to the primary ArUco
site. Each **landing mission controller** observes marker detections and tracks
its drone's role. The flight executor listens to the coordinator's retained
status and performs the authorized PX4 operations.

The mission controllers and coordinator are separate ROS 2 packages; `stdexec`
strategy evaluation runs in this package, not in those nodes.

## Mission Behavior

```text
                 Both drones airborne in Hold
                              |
                              v
                 CustomWaypoints approach
                    (independent routes)
                              |
                              v
                  Staging waypoint reached
                              |
                              v
                Coordinator assigns primary
                       /            \
                      /              \
                  PRIMARY           FOLLOWER
                     |                  |
             PrecisionLandCustom      PX4 Hold
                     |                  |
             ArUco search/align         |
                     |                  |
             Descend / touchdown        |
                     |                  |
             PX4 landing disarm         |
                     |                  |
             Coordinator COMPLETED -----+
                                        |
                                        v
                             stdexec eligibility checks
                             (concurrent, via when_all)
                                        |
                                        v
                                 POSITION_LANDING
                                        |
                                        v
                            Navigate to secondary pad
                                        |
                                        v
                             Descend / touchdown
                                        |
                                        v
                              PX4 landing disarm
```

The primary uses ArUco observations to align itself with a visual marker. The
follower does **not** attempt to land at that occupied marker; it waits for the
primary to complete and selects a separate position-landing site when eligible.
Both drones run interchangeable code and either can become primary.

### Mission-level execution and stdexec

The follower's strategy selector composes independent eligibility checks using
`stdexec::when_all`, with sender scheduling and result processing. Candidate
strategies include position landing, ArUco landing, and visual-estimator
landing, but **only `POSITION_LANDING` is eligible for actual follower execution
in the configured two-pad simulation**. Unavailable or unsafe alternatives
produce a safe `SAFE_HOLD` decision.

`stdexec` expresses application-level computation and its dependencies. **PX4
still owns the flight-control modes**; a strategy decision is verified again
before the executor activates a new mode. The selector does not directly
publish PX4 setpoints.

### Primary ArUco landing

1. `CustomWaypoints` executes four waypoints expressed in the shared `map`
   frame, resolved through TF2 into the vehicle's local NED frame.
2. After staging, a mission controller requests landing ownership.
3. The primary executor activates `PrecisionLandCustom`.
4. The mode performs `Search -> Approach -> Descend -> Finished` using ArUco
   target observations.
5. The executor reports successful completion only after PX4 has confirmed
   touchdown and landing-related disarm.

### Follower position landing

1. On primary `COMPLETED`, the follower reads the retained `LandingSiteMap`.
2. It checks site verification, occupancy, geometric separation, and the TF2
   transformation into its own `px4_N/odom_ned` frame.
3. `stdexec` selects `POSITION_LANDING` when eligible.
4. The existing `CustomWaypoints` mode navigates above the alternate pad and
   performs a controlled descent.
5. After fresh touchdown and landing-related disarm are confirmed, the
   executor publishes the mission ID on `landing_strategy/landed_mission`.
   The follower mission controller then transitions to `LANDED`.

The secondary site's `verified` flag is **operator-provided for this fixed
simulation world**; it is not an onboard perception guarantee.

## Build

This package uses C++20, ROS 2 Lyrical, the PX4 ROS 2 Interface Library,
Eigen3, TF2, and NVIDIA `stdexec`.

From the configured and sourced workspace root:

```bash
colcon build --packages-up-to precision_land_executor --symlink-install
source install/setup.zsh  # use setup.bash for Bash
```

## Run the Two-Drone Simulation

Use separate sourced terminals from the workspace root. The `resources`
package supplies PX4/Gazebo SITL and the shared TF tree.

**1. Start both PX4 vehicles and the competition world:**

```bash
ros2 launch resources simulation.launch.py \
  world:=aruco_competition \
  world_file:="$PWD/src/resources/worlds/aruco_competition.sdf" \
  num_vehicles:=2 \
  gz_model:=x500_mono_cam_down \
  urdf_model:=x500 \
  has_downward_camera:=true \
  vehicle_spacing:=3.0 \
  px4_autopilot_path:="$PWD/third_party/PX4-Autopilot"
```

**2. Bridge both cameras and start ArUco tracking:**

```bash
ros2 launch aruco_tracker two_drones_aruco.launch.py \
  num_vehicles:=2 \
  world_name:=aruco_competition \
  gz_model:=x500_mono_cam_down \
  camera_sensor_name:=camera \
  marker_size_m:=1.0
```

**3. Start the coordinator with both fixed simulation sites approved:**

```bash
ros2 launch landing_coordinator coordinator.launch.py \
  site_frame_id:=map \
  site_a_x:=10.0 site_a_y:=0.0 site_a_z:=0.0 site_a_verified:=true \
  site_b_x:=0.0 site_b_y:=-10.0 site_b_z:=0.0 site_b_verified:=true
```

**4. Register the two PX4 mode executors:**

```bash
ros2 launch precision_land_executor two_drone_precision_modes.launch.py \
  enable_auto_switch:=true \
  approach_enabled:=true \
  enable_position_landing:=true
```

**5. Start the mission controllers:**

```bash
ros2 launch landing_mission_controller mission_controllers.launch.py \
  require_approach_ready:=true \
  drone_a_delay_ms:=100 \
  drone_b_delay_ms:=300
```

In QGroundControl, **take off both drones and switch them to Hold** after PX4
mode registration completes. The waypoint approach starts when the executors
detect an airborne, armed vehicle in a suitable flight state. The launch does
**not** automatically take off both drones from the ground.

To favor Drone B in the simulation, start the mission controllers with
`drone_a_delay_ms:=120000 drone_b_delay_ms:=0`. This influences request timing;
it is not a hard-coded priority or a safety override.

## Simulation Sites and Coordinate Frames

| Site | Shared `map` coordinates (m) | Purpose |
| --- | --- | --- |
| `aruco_primary` | `(10, 0, 0)` | Vision-guided precision-landing marker. |
| `secondary_position` | `(0, -10, 0)` | Operator-approved independent position-landing pad. |

Drone A and B use different approach routes and staging altitudes. Their
`px4_0/odom_ned` and `px4_1/odom_ned` frames are linked to the shared `map`
frame through TF2. Landing-site positions are transformed into the **selected
drone's local frame** before navigation; they must not be reused directly as
local NED coordinates.

## Monitor the Mission

```bash
ros2 topic echo /landing_coordinator/status \
  landing_coordinator/msg/LandingStatus \
  --qos-reliability reliable --qos-durability transient_local

ros2 topic echo --once /px4_0/landing_strategy/decision \
  std_msgs/msg/String \
  --qos-reliability reliable --qos-durability transient_local

ros2 topic echo --once /px4_0/landing_strategy/landed_mission \
  std_msgs/msg/UInt64 \
  --qos-reliability reliable --qos-durability transient_local
```

A successful run reports primary completion, a follower strategy decision of
`POSITION_LANDING`, activation of the secondary route, touchdown, PX4
landing-related disarm, and `SECONDARY POSITION_LANDING completed`. The
follower controller transitions `READY_FOR_SELECTION -> LANDED` upon receiving
the matching local landing result.

## Tests

```bash
colcon test --packages-select precision_land_executor \
  --event-handlers console_direct+
colcon test-result --verbose
```

The tests cover strategy selection, site geometry and eligibility, TF2 site
resolution, waypoint-route validation, and follower policy behavior. Full
PX4/Gazebo landing validation additionally requires a SITL run.

## Design Contract

- Each drone retains its own PX4 flight-control authority and mode executor.
- Exclusive primary landing ownership comes from `landing_coordinator`;
  marker detection alone is not permission to descend.
- A follower's position-landing selection depends on a completed primary
  mission, a mission-matched site catalog, operator approval, geometric
  eligibility, and a valid TF2 transformation.
- If no eligible independent site exists, the selected strategy remains
  `SAFE_HOLD`. The strategy selector does not bypass PX4 safety checks.
- The completed demo covers **ArUco precision landing and secondary position
  landing**. It does not claim general-purpose autonomous landing-surface
  detection, decentralized consensus, or universal collision avoidance.

## Original Work and Attribution

Derived from the [Dronecode ROSCon 2025 PX4/ROS 2 workshop](https://github.com/Dronecode/roscon-25-workshop),
with the original workshop documentation retained in
[README.upstream.md](README.upstream.md) where available. Preserve its
attribution and applicable licensing requirements. This experimental extension
is not an official Dronecode workshop release.
