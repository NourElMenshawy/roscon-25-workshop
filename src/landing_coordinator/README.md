# Landing Coordinator — Shared Landing-Site Ownership

A ROS 2 (`rclcpp`) package that coordinates **exclusive access to the primary
landing site** in a two-drone PX4 simulation. The first eligible landing request
processed by the coordinator establishes the primary drone. Its owner and mission
identity are published to both vehicles, ensuring that the follower does not
attempt to land at the same ArUco marker.

Once the primary drone reports a confirmed landing, the coordinator publishes
the completed mission state and a **mission-scoped catalog of candidate landing
sites**. The follower can use this catalog to evaluate a separate landing site.

The coordinator manages mission ownership and shared information; it **does not
command PX4 flight modes, control either drone, or execute `stdexec` algorithms**.
Those responsibilities belong to `precision_land_executor`.

## Architecture

```text
      drone_a/landing_mission_controller     drone_b/landing_mission_controller
                      |                                  |
                      +--------- RequestLanding --------+
                                         |
                                         v
                            LandingCoordinator
                            +-----------------+
                            | LandingArbitrator|
                            | AVAILABLE        |
                            |   -> RESERVED    |
                            |   -> COMPLETED   |
                            +-----------------+
                                |           ^
                     LandingStatus          | ReportLandingComplete
                     (retained)            | (primary executor)
                                |
                                +--------------------+
                                |                    |
                    primary/follower roles    LandingSiteMap
                                              (on completion)
                                                    |
                                                    v
                                    Follower strategy evaluation
                                    in precision_land_executor
```

The primary landing reservation is retained after completion: a landed vehicle
must not be treated as having released that physical landing site. The alternate
site is a **separate, preconfigured candidate**, not a transferred reservation
for the occupied ArUco marker.

## ROS 2 Interfaces

| Name | Type | Purpose |
| --- | --- | --- |
| `/landing_coordinator/request_landing` | `landing_coordinator/srv/RequestLanding` | Request exclusive access to the ArUco site. |
| `/landing_coordinator/report_landing_complete` | `landing_coordinator/srv/ReportLandingComplete` | Report successful primary landing for the current owner and mission. |
| `/landing_coordinator/status` | `landing_coordinator/msg/LandingStatus` | Authoritative mission ownership and resource state. |
| `/landing_coordinator/landing_sites` | `landing_coordinator/msg/LandingSiteMap` | Candidate site catalog associated with the completed mission. |

`LandingStatus` defines three resource states:

- `AVAILABLE` (`0`): no owner; `mission_id=0`.
- `RESERVED` (`1`): the primary landing site belongs to `owner_id`.
- `COMPLETED` (`2`): the owner has reported a confirmed landing; the primary
  site remains occupied.

The status and site catalog are published with **reliable, transient-local QoS**
so late-joining subscribers can receive the latest mission information.

### Landing ownership

`RequestLanding.srv` accepts a `drone_id` and returns `granted`, `owner_id`,
`mission_id`, and `reason`. The first *processed* valid request wins; request
delays used in the simulation are not coordinator priorities. Repeated requests
from the owner are idempotent, while a competing drone cannot replace it.

### Completion and site catalog

`ReportLandingComplete.srv` accepts `mission_id`, `drone_id`, and `success`.
Only the current owner can successfully complete its mission; duplicate valid
completion reports are idempotent. The coordinator then publishes the
`COMPLETED` status and a `LandingSiteMap` containing:

- `aruco_primary`: the primary marker site, marked **occupied**;
- `secondary_position`: an alternate site, marked **unoccupied**.

Each site carries its ID, position, clearance radius, and `verified` flag. The
catalog also carries the coordinating `mission_id`, `owner_id`, and common
`frame_id`. It is published on primary mission completion, not on every
individual drone's landing.

**Site verification is explicit.** A `verified=true` flag means the operator
approved that fixed site for this particular simulation. It does not mean
perception has independently certified real terrain. The follower performs
additional site separation and TF2 checks before selecting position landing.

## Layout

- `srv/RequestLanding.srv` — landing ownership request and response.
- `srv/ReportLandingComplete.srv` — primary landing completion report.
- `msg/LandingStatus.msg` — retained ownership and completion state.
- `msg/LandingSite.msg`, `msg/LandingSiteMap.msg` — mission-scoped site catalog.
- `include/landing_coordinator/landing_arbitrator.hpp`, `src/landing_arbitrator.cpp`
  — thread-safe, ROS-independent ownership rules.
- `include/landing_coordinator/landing_coordinator.hpp`, `src/landing_coordinator.cpp`
  — ROS services, publications, and configured site data.
- `launch/coordinator.launch.py` — coordinator and site configuration.
- `launch/demo_arbitration.launch.py`, `src/mock_request_client.cpp` —
  standalone arbitration demonstration, without PX4.
- `test/` — arbitration and ROS integration tests.

## Build

From the configured, sourced ROS 2 Lyrical workspace:

```bash
colcon build --packages-up-to landing_coordinator --symlink-install
source install/setup.zsh  # use setup.bash if your shell is Bash
```

## Launch — Two-Drone Landing Demo

In the `aruco_competition` Gazebo world, the ArUco marker is at `(10, 0, 0)`
and the alternate position-landing pad is at `(0, -10, 0)` in the configured
shared `map` frame:

```bash
ros2 launch landing_coordinator coordinator.launch.py \
  site_frame_id:=map \
  site_a_x:=10.0 site_a_y:=0.0 site_a_z:=0.0 site_a_verified:=true \
  site_b_x:=0.0 site_b_y:=-10.0 site_b_z:=0.0 site_b_verified:=true
```

The launch file exposes `site_a_radius_m` and `site_b_radius_m` (both default to
`0.75`). All coordinates and verification flags default to conservative values;
**both site verification flags are false unless explicitly enabled**. Only use
this approved configuration with the corresponding simulation world.

The complete simulation also requires `resources`, `aruco_tracker`,
`precision_land_executor`, and `landing_mission_controller`; see the
`precision_land_executor` README for the end-to-end launch sequence.

## Inspect the Mission

```bash
ros2 topic echo /landing_coordinator/status \
  landing_coordinator/msg/LandingStatus \
  --qos-reliability reliable --qos-durability transient_local

ros2 topic echo --once /landing_coordinator/landing_sites \
  landing_coordinator/msg/LandingSiteMap \
  --qos-reliability reliable --qos-durability transient_local
```

The site catalog becomes available after a successful primary landing. For an
isolated coordinator test, inspect the service interfaces and submit ownership
requests:

```bash
ros2 interface show landing_coordinator/srv/RequestLanding
ros2 interface show landing_coordinator/srv/ReportLandingComplete

ros2 service call /landing_coordinator/request_landing \
  landing_coordinator/srv/RequestLanding '{drone_id: drone_a}'
ros2 service call /landing_coordinator/request_landing \
  landing_coordinator/srv/RequestLanding '{drone_id: drone_b}'
```

The first call should grant ownership to `drone_a`; the second should report
`reserved_by_other`. **Do not fabricate completion service calls while vehicles
are flying.** The PX4 mode executor reports real landing completion after
confirming touchdown and landing-related disarm.

A coordinator-only test without a PX4 simulation is also available:

```bash
ros2 launch landing_coordinator demo_arbitration.launch.py
```

## Tests

```bash
colcon test --packages-select landing_coordinator --event-handlers console_direct+
colcon test-result --verbose
```

The tests cover exclusive ownership, concurrent requests, idempotent retries,
owner/mission validation, completion state publication, and replay of retained
status and site data.

## Design Contract

- This is an **in-memory, single-primary-site, single-mission** coordinator for
  the current SITL demonstration, not a distributed consensus service.
- The resource is **not automatically released** when the primary lands;
  `COMPLETED` preserves ownership and occupancy.
- There is no cross-restart persistence or authentication of drone identities.
  A coordinator restart must **not** be treated as proof that the primary
  landing site is free.
- `mission_id` is coordinator-local, not a globally unique reservation token.
- A grant is not a flight-safety guarantee. The PX4 controller must enforce
  valid flight state, separation, and landing conditions.
- The coordinator does not run `stdexec`: concurrent strategy eligibility,
  selection, TF2 conversion, and PX4 execution live in the flight executor.

## Original Work and Attribution

This package is part of an experimental extension of the
[Dronecode ROSCon 2025 PX4/ROS 2 workshop](https://github.com/Dronecode/roscon-25-workshop).
It is not an official Dronecode workshop release. Preserve upstream
attribution and the applicable license notices when distributing the project.
