# UAV Offboard

Offboard package for drone behaviors and missions.

## Quickstart

0. Ensure that **PX4 is running** and connected via XRCE-DDS.

1. In a separate terminal,

```bash
ros2 launch uav_offboard launch.py
```

2. Set home

```bash
ros2 service call /uav/offboard_node/set_home std_srvs/srv/Trigger "{}"
```

3. Takeoff

```bash
ros2 action send_goal /uav/offboard_node/takeoff bb_uav_msgs/action/Takeoff "{altitude: 3.0, x_threshold: 0.1, y_threshold: 0.1, z_threshold: 0.1}" --feedback
```

4. Move

```bash
ros2 action send_goal /uav/offboard_node/go_to_position bb_uav_msgs/action/GoToPosition "{x: 3.0, y: 3.0, z: -2.0, relative: false, x_threshold: 0.1, y_threshold: 0.1, z_threshold: 0.1}" --feedback
```

5. Land / precision land / return to launch

Land:

```bash
ros2 service call /uav/offboard_node/land std_srvs/srv/Trigger "{}"
```

Precision land:

```bash
ros2 service call /uav/offboard_node/precision_landing std_srvs/srv/Trigger "{}"
```

Return to launch:

```bash
ros2 service call /uav/offboard_node/rtl std_srvs/srv/Trigger "{}"
```

## Usage

### Goal visualization

`/uav/goal_pose` shows the position target sent to PX4 while offboard position control is active. It uses the `odom_ned` frame, even for home-relative goals. The orientation is only a display placeholder; it does not command yaw.

```bash
ros2 topic echo /uav/goal_pose
```

### Home-relative coordinates

The supplied config enables `use_home_position`. Absolute goals (`relative: false`) are measured from PX4 home in north, east, down coordinates. For example, `(0, 0, -2)` is two metres above home. Relative goals are offsets from the UAV’s position when execution starts. Takeoff altitude is the distance to climb from that position.

Home odometry subtracts the home position from `/uav/odom_ned`, leaving orientation, velocity and covariance unchanged. It also publishes the `odom_ned -> home_ned` transform. Keep the existing raw odometry and body TF publishers running.

Home-relative absolute goals and home odometry need a finite PX4 home position with `valid_lpos: true`. Check that it is available:

```bash
ros2 topic echo /fmu/out/home_position_v1 px4_msgs/msg/HomePosition --once --qos-reliability best_effort
```

Some firmware uses `/fmu/out/home_position` without the version suffix. Set `home_position_topic` to match; the home odometry launch also accepts it as an argument. If the topic is missing, add this to the PX4 firmware's `src/modules/uxrce_dds_client/dds_topics.yaml` publications and rebuild with matching `px4_msgs`:

```yaml
- topic: /fmu/out/home_position
  type: px4_msgs::msg::HomePosition
```

Home changes apply to new goals. A running goal keeps its target, while feedback uses the latest home and reports NaN if home is unavailable. Disarming does not reset coordinates, and estimator resets are not compensated for.

The controls converter and planner must use the same frame as absolute goals. Use `/uav/odom_home_ned` and `home_ned` for home-relative control; keep localization and perception on raw odometry. To use raw PX4 coordinates for goals and feedback, set `use_home_position: false` and update the converter's `odom_topic`, `controls_frame` and planner frame checks together. Running the node without the supplied config defaults to raw coordinates.

### Takeoff

The vehicle can be in any mode for the `Takeoff` action but it must be armed and in "Offboard" flight mode for the `GoToPosition` action.

### Landing

Note that, by default, "Return to Launch" (RTL) returns the UAV to its **takeoff position** (not the position where the PX4 Flight Controller is turned on). This return position can be overridden by the `set_home` action.

For "Precision Landing" to work, add the following to `src/modules/uxrce_dds_client/dds_topics.yaml` in the `PX4-Autopilot` directory:

```yaml
- topic: /fmu/in/landing_target_pose
  type: px4_msgs::msg::LandingTargetPose
```

For sim, just run the `make` command again. For the real thing, you will need to flash the PX4 firmware.

## How It Works

1. The offboard node continuously publishes `OffboardControlMode` messages at 50 Hz to maintain offboard mode. It serves as a "heartbeat".
2. When a goal is received, it starts commanding the target position via `TrajectorySetpoint` messages.
3. It monitors the vehicle's current position from `VehicleLocalPosition` messages.
4. Feedback is published showing current position, distance to goal, and time remaining.
5. The action succeeds when all position errors are within their respective thresholds.
6. The action aborts if the timeout is reached before arriving at the target.
7. The action can be canceled at any time.
