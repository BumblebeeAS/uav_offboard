# UAV Offboard

Offboard package for drone behaviors and missions.

## Quickstart

0. Start PX4 and connect it via XRCE-DDS. Use `px4_msgs` matching the firmware.

1. Launch the nodes:

```bash
ros2 launch uav_offboard launch.py
```

2. Set home to the current position:

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
ros2 action send_goal /uav/offboard_node/land bb_uav_msgs/action/Land "{timeout: 60.0}" --feedback
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

`/uav/goal_pose` displays the position target sent to PX4 in `odom_ned`, including for home-relative goals. Its orientation does not command yaw.

```bash
ros2 topic echo /uav/goal_pose
```

### Home-relative coordinates

The supplied config enables `use_home_position`. Absolute goals (`relative: false`) use north, east, down coordinates relative to PX4 home: `(0, 0, -2)` is two metres above home. Relative goals use offsets from the position at execution start. Takeoff altitude is the climb distance from that position.

To publish `/uav/odom_home_ned` and the `odom_ned -> home_ned` transform, run the home odometry node alongside the existing raw odometry and body TF publishers:

```bash
ros2 launch uav_offboard home_odometry.launch.py
```

Home odometry subtracts the home position from `/uav/odom_ned`, preserving orientation, velocity and covariance.

Home-relative absolute goals and home odometry require finite home coordinates with `valid_lpos: true`. Check the home topic:

```bash
ros2 topic echo /fmu/out/home_position_v1 px4_msgs/msg/HomePosition --once --qos-reliability best_effort
```

Set `home_position_topic` to match your firmware; it may omit `_v1`. The home odometry launch also accepts this parameter as an argument. If the topic is missing, add it to the publications in PX4's `src/modules/uxrce_dds_client/dds_topics.yaml` and rebuild:

```yaml
- topic: /fmu/out/home_position
  type: px4_msgs::msg::HomePosition
```

Running goals keep their target when home changes. Feedback uses the latest home and reports NaN if it is unavailable. Disarming does not reset coordinates; estimator resets are not compensated for.

Home-relative odometry uses `/uav/odom_home_ned` and the `home_ned` frame. Set `use_home_position: false` to use raw PX4 coordinates for goals and feedback. Raw coordinates are the default without the supplied config.

### Takeoff

`Takeoff` sends arm and Offboard mode commands before climbing. `GoToPosition` requires the vehicle to be armed and in Offboard position control. Both require valid local position data.

### Landing

The `rtl` service sends PX4 a return-to-launch command. The `set_home` service sets PX4 home to the current position.

For precision landing, add the following to the subscriptions in PX4's `src/modules/uxrce_dds_client/dds_topics.yaml`:

```yaml
- topic: /fmu/in/landing_target_pose
  type: px4_msgs::msg::LandingTargetPose
```

Rebuild PX4 for simulation, or rebuild and flash the firmware for hardware.

## How It Works

The offboard node publishes `OffboardControlMode` at 50 Hz. During an active position goal, it sends `TrajectorySetpoint` commands while the vehicle is armed and in Offboard position control.

`Takeoff` and `GoToPosition` use `VehicleLocalPosition` to track progress and report position and distance feedback. They succeed when all axis errors meet their thresholds, and support cancellation. Neither has a goal timeout. The `Land` action accepts a timeout and completes when the vehicle disarms.

## Actuator control

The actuator node requires native MAVSDK 4 and a MAVLink 2 connection to PX4. Configure PX4 actuator output functions for `MAV_CMD_DO_SET_ACTUATOR`. The `actuation_indexes` parameter selects the actuators; ON sends `1` and OFF sends `-1`.
