# Landing Mission Controller — Per-Drone Mission Roles

A ROS 2 (`rclcpp`) package that manages each drone's **mission-level role** in
the coordinated PX4 landing demonstration. It observes ArUco detections,
requests exclusive access to the shared primary landing site, and tracks the
mission using the coordinator's authoritative status.

One controller is launched for each drone. The controllers do **not** directly
publish PX4 setpoints or activate flight modes. `precision_land_executor`
performs waypoint navigation, precision landing, follower Hold, and secondary
position landing based on the same coordinator state.

## Architecture

```text
/px4_0/target_pose             /px4_1/target_pose
/px4_0/approach/ready          /px4_1/approach/ready
       |                               |
       v                               v
/drone_a/landing_mission_controller   /drone_b/landing_mission_controller
       |                               |
       +------ RequestLanding ---------+
                       |
                       v
            /landing_coordinator
                       |
          LandingStatus (retained)
                       |
             +---------+---------+
             |                   |
       PRIMARY -> LANDED    FOLLOWER -> READY_FOR_SELECTION
                                         |
                               secondary landing result
                               /px4_N/landing_strategy/
                                   landed_mission
                                         |
                                         v
                                       LANDED
```

The controllers resolve **which drone is primary or follower**. They do not
implement the `stdexec` strategy selector; that pipeline runs in
`precision_land_executor` after primary mission completion.

## Mission State Machine

| State | Meaning |
| --- | --- |
| `SEARCHING` | Waiting for an eligible ArUco observation. |
| `WAITING_TO_REQUEST` | A valid observation was received; waiting for the configured request delay while observations remain fresh. |
| `REQUESTING` | An asynchronous landing-ownership request is outstanding. |
| `PRIMARY` | This drone owns the shared ArUco landing site. |
| `FOLLOWER` | Another drone owns the site; this drone must yield. |
| `READY_FOR_SELECTION` | The primary's mission is completed; the follower may select an independent landing strategy. |
| `LANDED` | The primary has completed its mission, or the follower has reported its own confirmed secondary landing. |

The coordinator's retained `LandingStatus` can assign a role even when a drone
has not itself received an ownership service response. The controller rejects
stale, conflicting, or malformed mission updates, and a completed secondary
landing cannot be undone by replayed coordinator status.

### Observation and ownership rules

- The `geometry_msgs/msg/PoseStamped` ArUco observation must contain finite
  position and orientation values, positive camera-optical depth, and a
  nonzero quaternion norm.
- With `require_approach_ready=true`, the controller ignores marker observations
  until the local `approach/ready` signal is true. This prevents competition
  from starting before the drone reaches its staging waypoint.
- The first valid request **processed by the coordinator** obtains primary
  ownership; request delays only influence demonstration timing.
- A controller never treats the absence of a service reply as authorization.
  Mission role is reconciled with the coordinator's authoritative status.
- Once the primary completes landing, the primary controller becomes `LANDED`
  and the follower becomes `READY_FOR_SELECTION`.
- The follower becomes `LANDED` only upon a matching, mission-scoped
  `landing_strategy/landed_mission` report from its own PX4 executor.

## ROS 2 Interfaces

For a drone in the `/px4_0` namespace, the launched controller uses:

| Direction | Name | Type | QoS / purpose |
| --- | --- | --- | --- |
| Subscribe | `/px4_0/target_pose` | `geometry_msgs/msg/PoseStamped` | Best effort; ArUco detection. |
| Subscribe | `/px4_0/approach/ready` | `std_msgs/msg/Bool` | Reliable, transient local; staging readiness. |
| Service client | `/landing_coordinator/request_landing` | `landing_coordinator/srv/RequestLanding` | Exclusive primary-site request. |
| Subscribe | `/landing_coordinator/status` | `landing_coordinator/msg/LandingStatus` | Reliable, transient local; mission role and completion. |
| Subscribe | `/px4_0/landing_strategy/landed_mission` | `std_msgs/msg/UInt64` | Reliable, transient local; confirmed secondary landing mission ID. |

Drone B uses the corresponding `/px4_1/...` topics. The approach readiness
subscription is created only when `require_approach_ready` is enabled.

## Layout

- `include/landing_mission_controller/mission_policy.hpp`,
  `src/mission_policy.cpp` — ROS-independent mission state machine.
- `include/landing_mission_controller/landing_mission_controller.hpp`,
  `src/landing_mission_controller.cpp` — ROS subscriptions, service client,
  timer, and mission transitions.
- `src/main.cpp` — ROS 2 executable.
- `launch/mission_controllers.launch.py` — both mission controllers.
- `launch/demo_mission_arbitration.launch.py`, `src/mock_target_publisher.cpp`
  — optional simulation-free mission arbitration exercise.
- `test/` — policy and ROS integration tests.

## Build

From the configured, sourced ROS 2 Lyrical workspace:

```bash
colcon build --packages-up-to landing_mission_controller --symlink-install
source install/setup.zsh  # use setup.bash for Bash
```

## Launch — Two-Drone Landing Demo

Launch the coordinator and both PX4 mode executors before starting the mission
controllers. For the completed competition demo, **enable staging readiness**:

```bash
ros2 launch landing_mission_controller mission_controllers.launch.py \
  require_approach_ready:=true \
  drone_a_delay_ms:=100 \
  drone_b_delay_ms:=300
```

The delay arguments affect how soon each eligible drone requests landing; the
winner is still determined by the coordinator's first processed valid request.
To favor Drone B in a repeatable demonstration:

```bash
ros2 launch landing_mission_controller mission_controllers.launch.py \
  require_approach_ready:=true \
  drone_a_delay_ms:=120000 \
  drone_b_delay_ms:=0
```

Both drones must first take off and enter PX4 Hold as described in the
`precision_land_executor` README. A valid ArUco observation and staging
readiness are still required for the request to be made; configuration delays
do not bypass those requirements.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `drone_id` | `""` (required) | Logical vehicle ID, e.g. `drone_a`. |
| `target_topic` | `target_pose` | Camera-relative marker observation topic. |
| `coordinator_service` | `/landing_coordinator/request_landing` | Ownership request service. |
| `status_topic` | `/landing_coordinator/status` | Retained mission status. |
| `request_delay_ms` | `100` (node) | Delay after the first valid observation before requesting ownership. |
| `require_approach_ready` | `false` | Gate marker eligibility on staging readiness; set `true` for this demo. |
| `approach_ready_topic` | `approach/ready` | Readiness topic when gating is enabled. |
| `secondary_landed_topic` | `landing_strategy/landed_mission` | Mission ID reported upon secondary touchdown/disarm. |
| `observation_timeout_ms` | `1500` | Maximum marker-observation age. |
| `response_timeout_ms` | `3000` | Bound on waiting for a request response. |

The two-drone launch file supplies per-drone topic names and defaults to
`drone_a_delay_ms=100` and `drone_b_delay_ms=300`.

## Inspect the Mission

```bash
ros2 node list | grep landing_mission_controller

ros2 topic echo /landing_coordinator/status \
  landing_coordinator/msg/LandingStatus \
  --qos-reliability reliable --qos-durability transient_local

ros2 topic echo --once /px4_0/landing_strategy/landed_mission \
  std_msgs/msg/UInt64 \
  --qos-reliability reliable --qos-durability transient_local
```

A typical completed run, with Drone B as primary, produces these controller
state changes:

```text
drone_b: SEARCHING -> WAITING_TO_REQUEST -> REQUESTING -> PRIMARY -> LANDED
drone_a: SEARCHING -> WAITING_TO_REQUEST -> FOLLOWER
         -> READY_FOR_SELECTION -> LANDED
```

A late-joining controller can learn the current reservation or completion
state from the retained coordinator status.

## Tests

```bash
colcon test --packages-select landing_mission_controller \
  --event-handlers console_direct+
colcon test-result --verbose
```

The tests cover ArUco eligibility, request scheduling, mission-state
transitions, ownership resolution, rejection of stale statuses, and
mission-matched secondary landing completion.

## Design Contract

- There is **one mission controller per drone**, not one global mission-policy
  node. The shared coordinator remains authoritative for primary ownership.
- The controller's states describe **mission roles**, not the drone's physical
  flight state. PX4 still controls flight-mode activation and setpoints.
- `READY_FOR_SELECTION` is a handoff to the flight executor's strategy logic;
  the controller does not decide or execute `POSITION_LANDING` itself.
- The `LANDED` state for secondary landing requires a matching reported mission
  ID from the local executor. The executor is responsible for independently
  checking touchdown and PX4 landing-related disarm.
- This is a fixed two-drone simulation configuration, not a general-purpose
  decentralized swarm controller.

## Original Work and Attribution

This package belongs to an experimental extension of the
[Dronecode ROSCon 2025 PX4/ROS 2 workshop](https://github.com/Dronecode/roscon-25-workshop).
It is not an official Dronecode workshop release. Retain applicable upstream
license and attribution notices.
