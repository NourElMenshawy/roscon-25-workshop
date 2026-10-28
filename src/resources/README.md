# PX4 ROS 2 Simulation Resources

This package provides the simulation resources and ROS 2 launch infrastructure used by the PX4 precision-landing and C++26 `std::execution` demonstrations.

It extends the resource infrastructure from the [Dronecode ROSCon 2025 Workshop](https://github.com/Dronecode/roscon-25-workshop), with a reorganized launch architecture developed for ROS 2 Lyrical.

The original workshop documentation is preserved in [README.upstream.md](README.upstream.md).

## Launch Architecture

The simulation is organized into separate launch files to distinguish simulation infrastructure, PX4 vehicle instances, ROS 2 integration, and visualization.

```text
simulation.launch.py
│
├── Gazebo simulation
│   └── gz_world.launch.py
│
├── PX4 simulation infrastructure
│   ├── px4_ros.launch.py
│   └── px4_vehicle.launch.py
│
├── ROS 2 vehicle integration
│   └── ros_vehicle.launch.py
│
└── Visualization
    └── visualization.launch.py
```

This diagram illustrates the intended separation of responsibilities; individual launch-file inclusion relationships should be verified against the implementation.

### Launch Files

| Launch file | Purpose |
|---|---|
| `simulation.launch.py` | Main entry point for the simulation |
| `gz_world.launch.py` | Gazebo world configuration and startup |
| `px4_ros.launch.py` | PX4 and ROS 2 simulation integration |
| `px4_vehicle.launch.py` | Individual PX4 vehicle instance configuration |
| `ros_vehicle.launch.py` | Vehicle-specific ROS 2 integration |
| `visualization.launch.py` | Visualization configuration |

## Simulation Components

The complete precision-landing demonstration combines the following components:

- **PX4 SITL:** Simulated flight controller and vehicle control.
- **Gazebo:** Physics simulation, vehicle models, and camera sensors.
- **uXRCE-DDS:** Communication between PX4 and ROS 2.
- **ArUco Tracker:** Marker detection and camera-relative target pose estimation.
- **PX4 TF:** Coordinate-frame visualization and transformations.
- **Precision Landing Executor:** Custom PX4 flight modes and target-processing logic.

The `resources` package provides the launch infrastructure and simulation assets. The application-specific functionality resides in the corresponding ROS 2 packages.

## Package Structure

```text
resources/
├── cfg/                # Bridge configuration
├── launch/             # Simulation launch files
├── meshes/             # Vehicle model assets
├── urdf/               # Robot descriptions
├── CMakeLists.txt
├── package.xml
├── README.md
└── README.upstream.md
```

## Running the Simulation

After building and sourcing the ROS 2 workspace:

```bash
ros2 launch resources simulation.launch.py
```

The simulation relies on the external PX4, Gazebo, and ROS 2 dependencies documented in the repository's main README.

## Multi-Vehicle Extension

The separation between simulation orchestration and individual vehicle launch files provides a foundation for the upcoming distributed-landing demonstration.

That extension will introduce multiple PX4 vehicle instances, separate ROS 2 namespaces, independent sensors, and distributed landing coordination.

## Attribution

This package is based on the simulation infrastructure of the Dronecode ROSCon 2025 Workshop.

The launch architecture has been reorganized and adapted for the experimental PX4/ROS 2 setup.

The original documentation and repository history are preserved to distinguish the upstream work from subsequent modifications.

This is an independent experimental extension, not an official Dronecode workshop release.
