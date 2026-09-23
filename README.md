# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/

## Navigation implementation

The robot package now contains the four-node navigation pipeline described by the ASD assignment. The planner's default 1.95 m clearance accounts for the supplied odometry spoof reporting the lidar frame: the SDF places that frame 0.8 m ahead of the 2.0 × 1.0 m chassis, giving an enclosing body radius of about 1.87 m plus margin. Re-measure this if the robot model or odometry frame changes:

```
/lidar -> costmap -> /costmap -> map_memory -> /map -> planner -> /path -> control -> /cmd_vel
                    ^                 ^                  ^                  ^
                 scan QoS          odometry           goal_point          odometry/lidar
```

The costmap ray-traces each valid LaserScan into raw free, occupied, and unknown cells. Map memory transforms those observations into a fixed `sim_world`-aligned occupancy grid using the odometry pose closest to the scan timestamp. The planner applies a conservative robot-clearance inflation around raw occupied cells and searches with eight-connected A*; unknown cells are traversable with an extra cost so the robot can explore. The controller follows the path with forward-only Pure Pursuit, rotates in place for large heading errors, and validates each command against fresh odometry, lidar, and the raw global map. Before publishing motion it sweeps the commanded differential-drive arc through the robot footprint, requires current lidar coverage for unknown future cells, enforces a stopping-distance margin, and publishes an explicit zero `Twist` when any input or safety check is unavailable.

The assignment's public interface uses `/goal_point` (`geometry_msgs/msg/PointStamped`). The supplied Foxglove layout also emits `/goal_pose` (`geometry_msgs/msg/PoseStamped`), which is accepted and converted to the same point goal. Goals must be expressed in the map frame published on `/map` (the simulator's usual frame is `sim_world`). The initial parameters are in each package's `config/params.yaml`; tune them only after confirming the simulator's robot footprint and scan behavior.

### Run locally or in WATcloud

Use the supplied `watod` workflow from a Linux shell or WSL. Copy `watod-config.sh` to `watod-config.local.sh` and set:

```
ACTIVE_MODULES="robot gazebo vis_tools"
```

Then build and launch:

```
./watod build
./watod up
```

Open the Foxglove endpoint printed by `watod up`, import the layout under `config/`, and inspect `/lidar`, `/costmap`, `/map`, and `/path`. Send a `geometry_msgs/msg/PointStamped` goal on `/goal_point` in the map frame. The navigation nodes are configured for simulation time by `bringup_robot/launch/robot.launch.py`.

For a focused rebuild while the other services are running:

```
./watod down robot
./watod build robot
./watod up robot
```

The controller intentionally stops on an empty or stale path, stale odometry/lidar/map data, a lidar return inside `emergency_distance`, an occupied swept-footprint cell, or insufficient scan coverage. Its safety parameters (`footprint_radius`, `collision_horizon`, `collision_margin`, `control_delay`, `braking_deceleration`, and acceleration limits) are conservative starting values for the supplied chassis and should be measured against the simulator before tuning. The planner republishes the active route every 500 ms and replans when the remembered map changes. A newly requested goal replaces the previous one.

### Verification checklist

Before submission, perform a clean build from a fresh checkout and verify:

- the ROS graph contains one `costmap`, `map_memory`, `planner`, `control`, and `odometry_spoof` node;
- `/map` is published at startup and persists after a turn;
- a reachable goal produces a visible `/path` and the robot stops within the configured goal tolerance;
- a static obstacle causes a detour without contact;
- an invalid or unreachable goal publishes an empty path and leaves `/cmd_vel` at zero;
- stopping or restarting the planner/controller leaves the simulated robot stopped until fresh inputs arrive.

Record the commit, parameters, goal coordinates, final error, and video evidence for the assignment submission.
