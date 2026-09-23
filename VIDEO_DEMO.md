# Video demonstration

The assignment submission asks for a GitHub code link and video proof. The supplied Foxglove layout is already configured for the required view and click-to-goal interaction.

## Start the demo

From Git Bash on Windows:

```bash
cd /c/Users/prana/Documents/Codex/2026-09-23/i-hav/work/wato_asd_training
ACTIVE_MODULES='robot gazebo vis_tools' BASE_PORT=18000 ./watod up
```

On Linux or WATcloud, use the normal `./watod up` command and use the Foxglove endpoint printed by the launcher. On this Windows setup, the bridge is exposed at `ws://localhost:18000`.

## Configure Foxglove

1. Open Foxglove Studio and connect to the bridge endpoint.
2. Import `config/wato_asd_training_foxglove_config .json`.
3. In the 3D panel, leave `/map`, `/costmap`, `/path`, and `/lidar` visible.
4. In the Image panel, select `/camera` if it is not already selected. The camera feed should be visible before recording.
5. Keep the 3D panel and Image panel visible together in the recording.

The 3D panel's publish configuration sends a map click as both `/goal_point` (`geometry_msgs/msg/PointStamped`) and `/goal_pose` (`geometry_msgs/msg/PoseStamped`) in the map frame. The planner accepts the point goal and the controller follows the resulting `/path` through `/cmd_vel`.

## Record the proof

Start recording before the click. Show the live camera feed, the robot and map, then click a reachable free-space point in the 3D map. Keep recording until the path is visible, the robot moves, and it stops at the goal. The video should show:

- `/camera` frames updating;
- `/map`, `/costmap`, `/path`, and the robot visible in the 3D panel;
- the click creating a goal and a nonempty path;
- the robot moving without contacting the static obstacles; and
- the robot stopping at the goal.

If the simulator has been restarted and the initial pose is inside the planner's conservative hard-clearance check, reset the robot to free space with the simulator's pose-reset service before recording. Do not silently move the clicked goal; the planner should publish an empty path and zero velocity for a genuinely blocked or invalid goal.
