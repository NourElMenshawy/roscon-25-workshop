# LandingCoordinator — exclusive landing ownership (milestone 1)

A ROS 2 (`rclcpp`) service node arbitrates **one shared landing resource**
between two or more simulated PX4 drones. The first valid request that the
coordinator processes wins. This package does **not** change PX4 flight modes,
perform landing, provide durable reservations, or run `std::execution` yet.

## Layout

- `srv/RequestLanding.srv` — ROS interface and grant response.
- `include/landing_coordinator/landing_arbitrator.hpp` — thread-safe ownership logic.
- `src/landing_arbitrator.cpp` — pure C++ implementation (no ROS dependency).
- `include/landing_coordinator/landing_coordinator.hpp`, `src/landing_coordinator.cpp`
  — ROS service node.
- `src/main.cpp` — coordinator executable.
- `src/mock_request_client.cpp` — simple demo requester, not a PX4 integration.
- `launch/coordinator.launch.py` — launch coordinator alone.
- `launch/demo_arbitration.launch.py` — launch coordinator and two mock requesters.
- `test/` — pure arbitration tests and ROS service integration tests.

## Build

From your **existing sourced ROS 2 workspace** (e.g. ROS Lyrical):

```bash
colcon build --packages-select landing_coordinator --symlink-install
source install/setup.bash
```

## Run the node

```bash
ros2 launch landing_coordinator coordinator.launch.py
```

In another sourced terminal:

```bash
ros2 service list | grep request_landing
ros2 interface show landing_coordinator/srv/RequestLanding
ros2 service call /landing_coordinator/request_landing \
  landing_coordinator/srv/RequestLanding "{drone_id: drone_a}"
ros2 service call /landing_coordinator/request_landing \
  landing_coordinator/srv/RequestLanding "{drone_id: drone_b}"
ros2 service call /landing_coordinator/request_landing \
  landing_coordinator/srv/RequestLanding "{drone_id: drone_a}"
```

Expected results, in order:

1. `drone_a`: `granted=true`, `owner_id=drone_a`, `mission_id=1`, `reason=granted`.
2. `drone_b`: `granted=false`, `owner_id=drone_a`, `mission_id=1`, `reason=reserved_by_other`.
3. `drone_a`: `granted=true`, `owner_id=drone_a`, `mission_id=1`, `reason=already_owner`.

The service is private to the node. With the default node name, it resolves to
`/landing_coordinator/request_landing`.

## Two-requester demo (no PX4/Gazebo)

```bash
ros2 launch landing_coordinator demo_arbitration.launch.py
```

This starts the coordinator and two mock clients. A waits 100 ms after service
discovery, B waits 300 ms. A should usually win, but **request timing is not an
ownership priority**: the first *processed* eligible request wins. Both mock
clients exit after printing their responses; the coordinator keeps running.

## Automated tests

```bash
cd ~/workspace/px4_ros_wc
colcon build --packages-select landing_coordinator --symlink-install
colcon test --packages-select landing_coordinator --event-handlers console_direct+
colcon test-result --verbose
```

The tests cover:

- First valid request wins.
- A different drone cannot replace the owner.
- Same-drone retries are idempotent.
- Blank drone identities cannot reserve the resource.
- 32 simultaneous pure-C++ requests yield exactly one winner.
- ROS service requests behave correctly with two outstanding requests.

## Design contract and deliberate limitations

- Only one marker/landing resource and one mission attempt are modeled.
- Ownership never expires or transfers automatically. Restarting the node
  creates a **new, independent** coordinator state. **Never use restart as
  authority to permit a second vehicle to descend during an active landing.**
- The arbitrator is thread-safe, but does not authenticate drone identities.
- `mission_id=1` is unique only in one in-memory coordinator instance; it is
  **not** a globally unique mission/lease token.
- A grant is permission from this demonstration coordinator, **not** a flight
  safety guarantee. PX4 mission logic, geofencing, valid navigation estimates,
  and separation checks remain essential.
- The package includes no auto-release on landing, timeout, or missed heartbeat.
- The flight-side client must never engage `PrecisionLandCustom` based solely
  on ArUco detection; it must first obtain a valid grant.
- With an active grant, all other drones must remain in an independently safe
  flight mode (e.g. Hold). A client must not treat lack of response as a grant.

## Next milestone

Connect per-drone ROS 2 mission controllers to this service. The winning drone
will be allowed to request its existing `PrecisionLandCustom` mode; the losing
drone remains in Hold. We will define completion messages and follower
algorithm-selection **after** validating the arbitration contract.

Replace the example maintainer details in `package.xml` before publishing.
