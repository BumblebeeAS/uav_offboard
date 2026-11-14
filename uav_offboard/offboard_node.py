#!/usr/bin/env python3
import math
import time
from typing import Callable

import numpy as np
import rclpy
from bb_uav_msgs.action import CircleSweep, SpiralSearch, GoToPosition, Land, Takeoff
from bb_uav_msgs.msg import Geofence, GoToFeedback, GoToResult
from frames.utils.geofence import DEFAULT_GEOFENCE_MARGIN, geofence_region, spiral_region
from pygeodesy.ltp import LocalCartesian
from shapely.affinity import affine_transform
from shapely.geometry import LineString, Point
from bb_uav_msgs.srv import ChangeOffboardControlMode
from geometry_msgs.msg import PoseStamped, Vector3
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    HomePosition,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from uav_offboard.utils.circle_sweep import sweep_offset, sweep_total_duration
from uav_offboard.utils.spiral_search import SpiralPath
from uav_offboard.utils.goto import GeneralGoal, is_acceleration_valid
from uav_offboard.utils.offboard_mode import OffboardMode
from uav_offboard.utils.qos_profiles import QOS_PROFILE_HOME, QOS_PROFILE_PUB, QOS_PROFILE_SUB


class OffboardNode(Node):
    """ROS node to interface with PX4 offboard control via action servers and services."""

    def __init__(self):
        super().__init__("offboard_node")

        # Parameters
        goal_pose_topic = self.declare_parameter("goal_pose_topic", "goal_pose").value
        self.goal_pose_frame = self.declare_parameter("goal_pose_frame", "odom_ned").value
        self.use_home_position = self.declare_parameter(
            "use_home_position", False
        ).value
        home_position_topic = self.declare_parameter(
            "home_position_topic", "/fmu/out/home_position_v1"
        ).value
        vehicle_status_topic = (
            self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status")
            .get_parameter_value()
            .string_value
        )
        vehicle_local_position_topic = (
            self.declare_parameter(
                "vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1"
            )
            .get_parameter_value()
            .string_value
        )
        offboard_control_mode_topic = (
            self.declare_parameter(
                "offboard_control_mode_topic", "/fmu/in/offboard_control_mode"
            )
            .get_parameter_value()
            .string_value
        )
        trajectory_setpoint_topic = (
            self.declare_parameter(
                "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint"
            )
            .get_parameter_value()
            .string_value
        )
        vehicle_command_topic = (
            self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")
            .get_parameter_value()
            .string_value
        )
        acceleration_control_topic = (
            self.declare_parameter(
                "acceleration_control_topic", "/uav/acceleration_control"
            )
            .get_parameter_value()
            .string_value
        )

        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        self._geofence = self._spiral_origin = None
        self.spiral_geofence_margin = self.declare_parameter(
            'spiral_geofence_margin', DEFAULT_GEOFENCE_MARGIN).value
        self.spiral_geofence_max_age = self.declare_parameter('spiral_geofence_max_age', 5.).value
        if (not math.isfinite(self.spiral_geofence_margin) or self.spiral_geofence_margin < 0
                or not math.isfinite(self.spiral_geofence_max_age) or self.spiral_geofence_max_age <= 0):
            raise ValueError('Invalid spiral geofence margin or maximum age')
        self.geofence_sub = self.create_subscription(
            Geofence, 'geofence', self.geofence_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE), callback_group=self.callback_group)

        # Subscribers
        self.home_sub = self.create_subscription(
            HomePosition, home_position_topic, self.home_position_callback,
            QOS_PROFILE_HOME,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            vehicle_status_topic,
            self.vehicle_status_callback,
            QOS_PROFILE_SUB,
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            vehicle_local_position_topic,
            self.local_position_callback,
            QOS_PROFILE_SUB,
        )
        self.acceleration_sub = self.create_subscription(
            Vector3,
            acceleration_control_topic,
            self.acceleration_callback,
            QOS_PROFILE_SUB,
        )

        # Publishers
        self.publisher_goal_pose = self.create_publisher(PoseStamped, goal_pose_topic, 10)
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, offboard_control_mode_topic, QOS_PROFILE_PUB
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, trajectory_setpoint_topic, QOS_PROFILE_PUB
        )
        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand, vehicle_command_topic, QOS_PROFILE_PUB
        )

        # Continuously publish heartbeat and traj setpoint.
        # PX4 requires that the vehicle is already receiving
        # OffboardControlMode messages before it will arm in offboard mode,
        # or before it will switch to offboard mode when flying
        self.timer_ = self.create_timer(
            0.02, self.control_loop_callback, callback_group=self.callback_group
        )

        # Vehicle state
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.position_valid = False
        self.home_position = None
        self.offboard_mode = OffboardMode()

        # Service servers
        self.set_home_service_ = self.create_service(
            Trigger, "~/set_home", self.set_home_callback
        )
        self.rtl_service_ = self.create_service(Trigger, "~/rtl", self.rtl_callback)
        self.prec_landing_service_ = self.create_service(
            Trigger, "~/precision_landing", self.precision_landing_callback
        )
        self.change_offboard_mode_service_ = self.create_service(
            ChangeOffboardControlMode,
            "~/change_offboard_mode",
            self.change_offboard_mode_callback,
        )

        # Action servers
        self._goto_position_action_server = ActionServer(
            self,
            GoToPosition,
            "~/go_to_position",
            execute_callback=self.execute_goto_position_callback,
            goal_callback=self.goto_position_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self._circle_sweep_action_server = ActionServer(
            self, CircleSweep, "~/circle_sweep",
            execute_callback=self.execute_circle_sweep_callback,
            goal_callback=self.circle_sweep_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self._spiral_search_action_server = ActionServer(
            self, SpiralSearch, "~/spiral_search",
            execute_callback=self.execute_spiral_search_callback,
            goal_callback=self.spiral_search_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self._takeoff_action_server = ActionServer(
            self,
            Takeoff,
            "~/takeoff",
            execute_callback=self.execute_takeoff_callback,
            goal_callback=self.takeoff_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self._land_action_server = ActionServer(
            self,
            Land,
            "~/land",
            execute_callback=self.execute_land_callback,
            goal_callback=self.land_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # Goal tracking
        self.absolute_target = None  # Stores the computed absolute target position
        self.latest_acceleration_msg = Vector3(x=0.0, y=0.0, z=0.0)
        self.is_goal_active = False

        self.get_logger().info("OffboardNode started")

    # -------------------- Vehicle Command Helpers --------------------

    def publish_vehicle_command(self, command, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)

    def set_home_location(self):
        """
        set home position to current location
        Changes the home location either to the current location or a specified location.
        |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|
        """
        # Debug need to check whether home location works
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_HOME, param1=1.0)
        self.get_logger().info("Set home location...")

    def rtl(self):
        """Return to launch command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("RTL command sent....")

    def arm(self):
        """Arm drone. Param1=1.0 for arm."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent....")

    def disarm(self):
        """Disarm drone. Param1=0.0 for disarm."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent....")

    def engage_offboard_mode(self):
        """Switch mode to offboard mode"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Switching to offboard mode....")

    def engage_land_mode(self):
        """
        Land at location.
        |Unused|Unused|Unused|Desired yaw angle.|Latitude|Longitude|Altitude|
        """
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Sent Land command....")

    def vehicle_status_callback(self, msg: VehicleStatus):
        """Update vehicle navigation and arming state"""
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def local_position_callback(self, msg: VehicleLocalPosition):
        """Update current position from vehicle"""
        self.current_position = np.array([msg.x, msg.y, msg.z])
        self._position_received_at = time.monotonic()
        self.position_valid = bool(
            msg.xy_valid and msg.z_valid and np.isfinite(self.current_position).all()
        )
        reference = (msg.ref_lat, msg.ref_lon, msg.ref_timestamp,
                     msg.xy_reset_counter, msg.z_reset_counter)
        valid_reference = (msg.xy_global and msg.ref_timestamp > 0
                           and math.isfinite(msg.ref_lat) and math.isfinite(msg.ref_lon)
                           and -90 <= msg.ref_lat <= 90 and -180 <= msg.ref_lon <= 180)
        # Freeze matching coordinates/reference together across callbacks.
        self._spiral_origin = ((reference, self.current_position.copy())
                               if valid_reference and self.position_valid else None)

    def home_position_callback(self, msg: HomePosition):
        """Track PX4 home without changing an already resolved target."""
        home = np.array([msg.x, msg.y, msg.z])
        self.home_position = (
            home if msg.valid_lpos and np.isfinite(home).all() else None
        )

    @property
    def feedback_position(self):
        """Report NED position in the configured absolute-goal frame."""
        if not self.use_home_position:
            return self.current_position
        home = self.home_position
        if home is None:
            return np.full(3, np.nan)
        return self.current_position - home

    def resolve_target(self, goal: GeneralGoal):
        """Validate references and convert a goal to the PX4 local NED frame."""
        if not self.position_valid:
            raise ValueError("Failed to get valid position data")
        target = np.array([goal.x, goal.y, goal.z])
        if goal.relative:
            return self.current_position + target
        if self.use_home_position:
            home = self.home_position
            if home is None:
                raise ValueError("No valid PX4 home position received")
            return home + target
        return target

    def acceleration_callback(self, msg: Vector3):
        """Update latest acceleration command from topic"""
        self.latest_acceleration_msg = msg
        self.latest_acceleration_msg_time = self.get_clock().now()

    def control_loop_callback(self):
        """High-rate control loop for publishing offboard commands"""
        # Always publish offboard control mode to keep the connection alive
        offboard_msg = OffboardControlMode()
        current_mode = self.offboard_mode.get_mode()
        offboard_msg.position = current_mode["position"]
        offboard_msg.velocity = current_mode["velocity"]
        offboard_msg.acceleration = current_mode["acceleration"]
        offboard_msg.attitude = current_mode["attitude"]
        offboard_msg.body_rate = current_mode["body_rate"]
        offboard_msg.thrust_and_torque = current_mode["thrust_and_torque"]
        offboard_msg.direct_actuator = current_mode["direct_actuator"]

        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(offboard_msg)

        if (
            self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        ):
            if (
                self.is_goal_active
                and self.absolute_target is not None
                and self.offboard_mode.is_position
            ):
                self.publish_trajectory_setpoint()

            if self.offboard_mode.is_acceleration and is_acceleration_valid(
                self.get_clock().now(), self.latest_acceleration_msg_time, 1.0
            ):
                self.publish_acceleration_setpoint()

    def publish_trajectory_setpoint(self):
        """Publish one coherent target despite concurrent action cleanup."""
        # Actions and the timer use a reentrant callback group. Cleanup can clear
        # absolute_target after control_loop_callback checks it, or between XYZ
        # reads here. Capture the array reference once and use one snapshot.
        target = self.absolute_target
        if target is None:
            return
        target = target.copy()
        trajectory_msg = TrajectorySetpoint()
        now = self.get_clock().now()
        trajectory_msg.timestamp = int(now.nanoseconds / 1000)
        trajectory_msg.position[0] = target[0]
        trajectory_msg.position[1] = target[1]
        trajectory_msg.position[2] = target[2]
        trajectory_msg.yaw = float("nan")  # Let PX4 handle yaw
        self.publisher_trajectory.publish(trajectory_msg)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = now.to_msg()
        goal_pose.header.frame_id = self.goal_pose_frame
        goal_pose.pose.position.x = float(trajectory_msg.position[0])
        goal_pose.pose.position.y = float(trajectory_msg.position[1])
        goal_pose.pose.position.z = float(trajectory_msg.position[2])
        # Visualization placeholder: this controller does not command orientation.
        goal_pose.pose.orientation.w = 1.0
        self.publisher_goal_pose.publish(goal_pose)

    def publish_acceleration_setpoint(self):
        """Publish acceleration setpoint"""
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        trajectory_msg.position[0] = float("nan")
        trajectory_msg.position[1] = float("nan")
        trajectory_msg.position[2] = float("nan")

        trajectory_msg.velocity[0] = float("nan")
        trajectory_msg.velocity[1] = float("nan")
        trajectory_msg.velocity[2] = float("nan")

        trajectory_msg.acceleration[0] = self.latest_acceleration_msg.x
        trajectory_msg.acceleration[1] = self.latest_acceleration_msg.y
        trajectory_msg.acceleration[2] = self.latest_acceleration_msg.z
        trajectory_msg.yaw = float("nan")  # Let PX4 handle yaw
        self.publisher_trajectory.publish(trajectory_msg)

    # -------------------- Service Callbacks --------------------

    def set_home_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """
        Service callback for set home location request.

        Args:
            request: Trigger.Request
            response: Trigger.Response with success and message fields

        Returns:
            response: Trigger.Response
        """
        try:
            self.get_logger().info("Set home service called")
            self.set_home_location()

            response.success = True
            response.message = "Home location set to current position."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Set home failed: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def rtl_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """
        Service callback for return to launch request.

        Args:
            request: Trigger.Request
            response: Trigger.Response with success and message fields

        Returns:
            response: Trigger.Response
        """
        try:
            self.get_logger().info("RTL service called")
            self.rtl()

            response.success = True
            response.message = "RTL command sent."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"RTL failed: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def precision_landing_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """
        Service callback for precision landing request.

        Args:
            request: Trigger.Request
            response: Trigger.Response with success and message fields

        Returns:
            response: Trigger.Response
        """
        try:
            self.get_logger().info("Precision landing service called")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_PRECLAND)

            response.success = True
            response.message = "Precision landing command sent."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Precision landing failed: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def change_offboard_mode_callback(
        self,
        request: ChangeOffboardControlMode.Request,
        response: ChangeOffboardControlMode.Response,
    ) -> ChangeOffboardControlMode.Response:
        """
        Service callback for changing offboard control mode.

        Args:
            request: ChangeOffboardControlMode.Request
            response: ChangeOffboardControlMode.Response
        Returns:
            response: ChangeOffboardControlMode.Response
        """
        try:
            self.get_logger().info("Change offboard mode service called")
            self.offboard_mode.set_mode(
                request.position,
                request.velocity,
                request.acceleration,
                request.attitude,
                request.body_rate,
                request.thrust_and_torque,
                request.direct_actuator,
            )
            response.success = True

        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to change offboard mode: {str(e)}")

        return response

    # -------------------- Action Server Callbacks --------------------

    def validate_goal(self, goal: GeneralGoal) -> bool:
        # Check now; execution resolves again using the latest reference.
        try:
            self.resolve_target(goal)
        except ValueError as exc:
            self.get_logger().warning(f"{exc}; rejecting goal")
            return False
        duration = getattr(goal, "duration", 0.0)
        if not math.isfinite(duration) or duration < 0.0:
            self.get_logger().warning("Invalid goto duration; must be finite and nonnegative")
            return False
        if goal.x_threshold <= 0 or goal.y_threshold <= 0 or goal.z_threshold <= 0:
            self.get_logger().warning("Invalid thresholds, must be positive")
            return False

        if self.is_goal_active:
            self.get_logger().warning("Another goal is in progress; rejecting...")
            return False

        return True

    def goto_position_goal_callback(self, goal_request: GoToPosition.Goal):
        """Accept or reject a GoToPosition goal"""
        mode = "relative" if goal_request.relative else "absolute"
        self.get_logger().info(
            f"Received goal request ({mode}): target=({goal_request.x}, {goal_request.y}, {goal_request.z})"
        )

        is_goal_valid = self.validate_goal(GeneralGoal.from_position_goal(goal_request))
        return GoalResponse.ACCEPT if is_goal_valid else GoalResponse.REJECT

    def _sweep_position_valid(self):
        received = getattr(self, "_position_received_at", None)
        return (self.position_valid and received is not None
                and time.monotonic() - received <= 1.0)

    def hold_current_position(self, altitude=None):
        """Replace the last moving target before acknowledging cancellation."""
        if (self._sweep_position_valid()
                and getattr(self, 'nav_state', None) == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and getattr(self, 'arming_state', None) == VehicleStatus.ARMING_STATE_ARMED):
            self.absolute_target = self.current_position.copy()
            if altitude is not None:
                self.absolute_target[2] = altitude
            self.publish_trajectory_setpoint()

    def circle_sweep_goal_callback(self, request):
        values = (request.radius, request.duration,
                  request.return_threshold, request.return_timeout)
        if (not all(math.isfinite(v) and v > 0.0 for v in values)
                or not self._sweep_position_valid() or self.is_goal_active
                or not self.offboard_mode.is_position):
            self.get_logger().warning("Rejecting sweep: invalid parameters/state or busy")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def execute_circle_sweep_callback(self, goal_handle):
        """Run one timed circle, then require measured arrival at its endpoint.

        Freeze the current position once. Offsets are in horizontal local
        NED, so changing odometry never shifts the circle centre or its altitude.
        Use the normal action cancellation path rather than a competing PX4
        setpoint publisher in the tree process.
        """
        request = goal_handle.request
        result = GoToResult(success=False)
        rate = None
        self.is_goal_active = True
        try:
            if not self._sweep_position_valid():
                raise ValueError("Sweep requires fresh valid local position")
            endpoint = self.current_position.copy()
            centre = endpoint.copy()
            if request.start_on_circle:
                # The caller already reached the +north edge with a normal goto.
                # Freeze geometry at acceptance so odometry updates cannot move it.
                centre[0] -= request.radius
            self.absolute_target = endpoint.copy()
            start = self.get_clock().now()
            trajectory_duration = sweep_total_duration(
                request.radius, request.duration, request.start_on_circle)
            rate = self.create_rate(50)
            self.get_logger().info(
                f"Circle sweep: radius={request.radius:.3f} m, "
                f"revolution={request.duration:.2f} s, centre={centre.tolist()}, "
                f"start_on_circle={request.start_on_circle}")
            while rclpy.ok():
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                result.time_elapsed = elapsed
                result.final_x, result.final_y, result.final_z = self.feedback_position
                if goal_handle.is_cancel_requested:
                    result.message = "Circle sweep canceled"
                    goal_handle.canceled()
                    return CircleSweep.Result(result=result)
                if not self._sweep_position_valid():
                    raise ValueError("Lost fresh valid position during circle sweep")
                self.absolute_target = centre + sweep_offset(
                    elapsed, request.radius, request.duration, request.start_on_circle)
                distance = float(np.linalg.norm(self.absolute_target - self.current_position))
                # Report tracking error without aborting on normal controller lag.
                # Completion still requires a measured return to the frozen endpoint.
                goal_handle.publish_feedback(CircleSweep.Feedback(feedback=GoToFeedback(
                    current_x=float(self.feedback_position[0]),
                    current_y=float(self.feedback_position[1]),
                    current_z=float(self.feedback_position[2]), distance_to_goal=distance)))
                if elapsed >= trajectory_duration:
                    if np.all(np.abs(self.current_position - endpoint) <= request.return_threshold):
                        result.success = True
                        result.message = "Circle sweep complete; returned to endpoint"
                        goal_handle.succeed()
                        return CircleSweep.Result(result=result)
                    if elapsed > trajectory_duration + request.return_timeout:
                        raise ValueError("Circle sweep timed out returning to endpoint")
                rate.sleep()
            raise ValueError("Circle sweep interrupted by shutdown")
        except Exception as exc:
            result.message = str(exc)
            self.get_logger().warning(result.message)
            goal_handle.abort()
            return CircleSweep.Result(result=result)
        finally:
            self.reset_internal_state()
            if rate is not None:
                self.destroy_rate(rate)

    def geofence_callback(self, message):
        self._geofence = message

    def _fresh_spiral_geofence(self):
        fence, origin = self._geofence, self._spiral_origin
        if fence is None or origin is None:
            raise ValueError('Spiral requires a geofence and a valid global PX4 local reference')
        stamp = fence.stamp.sec + fence.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not 0 <= age <= self.spiral_geofence_max_age:
            raise ValueError('Spiral geofence is stale or future-dated')
        return fence, origin

    def _spiral_region(self, fence, reference, centre):
        projection = LocalCartesian(reference[0], reference[1], 0.)
        region = geofence_region(fence, projection)
        # PX4 positions are NED; the shared fence geometry is ENU.
        return spiral_region(region, Point(centre[1], centre[0]), self.spiral_geofence_margin)

    def _spiral_path(self, request, fence, reference, centre):
        safe = self._spiral_region(fence, reference, centre)
        relative = affine_transform(safe, [0., 1., 1., 0., -centre[0], -centre[1]])
        path = SpiralPath(request.radius, request.turn_spacing, request.speed, region=relative)
        route = affine_transform(LineString(path.points[:, :2]),
                                 [0., 1., 1., 0., centre[1], centre[0]])
        if not safe.covers(route):
            raise ValueError('Planned spiral violates the geofence clearance')
        return path, safe, route

    def spiral_search_goal_callback(self, request):
        values = (request.radius, request.turn_spacing, request.speed,
                  request.settle_threshold, request.settle_timeout)
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or request.turn_spacing > 2 * request.radius
                or not self._sweep_position_valid() or self.is_goal_active
                or not self.offboard_mode.is_position):
            self.get_logger().warning("Rejecting spiral: invalid parameters/state or busy")
            return GoalResponse.REJECT
        try:
            fence, (reference, centre) = self._fresh_spiral_geofence()
            self._spiral_region(fence, reference, centre)
        except ValueError as exc:
            self.get_logger().warning(f'Rejecting spiral: {exc}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def execute_spiral_search_callback(self, goal_handle):
        """Use the normal Offboard publisher and drain cancellation before approach."""
        request = goal_handle.request
        result = GoToResult(success=False)
        rate = None
        self.is_goal_active = True
        try:
            if not self._sweep_position_valid():
                raise ValueError("Spiral requires fresh valid local position")
            fence, (reference, centre) = self._fresh_spiral_geofence()
            path, safe_region, route = self._spiral_path(request, fence, reference, centre)
            endpoint = centre + path.offset(path.duration)
            start = self.get_clock().now()
            rate = self.create_rate(50)
            self.absolute_target = centre.copy()
            self.get_logger().info(
                f"Spiral search: radius={request.radius:g} m, clipped={path.clipped}, "
                f"geofence clearance={self.spiral_geofence_margin:g} m, spacing={request.turn_spacing:g} m, "
                f"speed<={request.speed:g} m/s, duration={path.duration:.1f} s")
            while rclpy.ok():
                elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
                result.time_elapsed = elapsed
                result.final_x, result.final_y, result.final_z = self.feedback_position
                if goal_handle.is_cancel_requested:
                    # Replace the moving target with measured XY before acknowledging.
                    # Keep the search altitude; don't continue toward an old spiral point.
                    origin = self._spiral_origin
                    self.hold_current_position(altitude=centre[2] if origin and origin[0] == reference else None)
                    result.message = "Spiral search canceled; holding current XY"
                    goal_handle.canceled()
                    break
                if not self._sweep_position_valid():
                    raise ValueError("Lost fresh valid position during spiral")
                current_fence, (current_reference, _) = self._fresh_spiral_geofence()
                if current_reference != reference:
                    raise ValueError('PX4 position reference/reset changed during spiral')
                if current_fence is not fence:
                    safe_region = self._spiral_region(current_fence, reference, centre)
                    if not safe_region.covers(route):
                        raise ValueError('Updated geofence no longer contains the spiral')
                    fence = current_fence
                self.absolute_target = centre + path.offset(elapsed)
                distance = float(np.linalg.norm(self.absolute_target - self.current_position))
                goal_handle.publish_feedback(SpiralSearch.Feedback(feedback=GoToFeedback(
                    current_x=float(self.feedback_position[0]),
                    current_y=float(self.feedback_position[1]),
                    current_z=float(self.feedback_position[2]), distance_to_goal=distance)))
                if elapsed >= path.duration:
                    if np.all(np.abs(self.current_position - endpoint) <= request.settle_threshold):
                        result.success = True
                        result.message = "Spiral search complete"
                        goal_handle.succeed()
                        break
                    if elapsed > path.duration + request.settle_timeout:
                        raise ValueError("Spiral timed out settling at endpoint")
                rate.sleep()
            else:
                raise ValueError("Spiral interrupted by shutdown")
        except Exception as exc:
            result.message = str(exc)
            self.get_logger().warning(result.message)
            self.hold_current_position()
            goal_handle.abort()
        finally:
            self.reset_internal_state()
            if rate is not None:
                self.destroy_rate(rate)
        return SpiralSearch.Result(result=result)

    def takeoff_goal_callback(self, goal_request: Takeoff.Goal):
        """Accept or reject a Takeoff goal"""
        self.get_logger().info(
            f"Received takeoff request: altitude={goal_request.altitude:.2f} m"
        )

        if goal_request.altitude <= 0:
            self.get_logger().warning("Invalid altitude: must be positive meters")
            return GoalResponse.REJECT

        is_valid_goal = self.validate_goal(GeneralGoal.from_takeoff_goal(goal_request))
        return GoalResponse.ACCEPT if is_valid_goal else GoalResponse.REJECT

    def land_goal_callback(self, goal_request: Takeoff.Goal):
        """Accept or reject a Land goal (using Takeoff action type)"""
        self.get_logger().info("Received land request")

        if self.is_goal_active:
            self.get_logger().warning("Another goal is in progress; rejecting...")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(
        self,
        goal_handle,
        goal: GeneralGoal,
        publish_feedback_fn: Callable[[GoToFeedback], None],
    ) -> GoToResult:
        """Execute the goal"""
        self.is_goal_active = True
        start_time = self.get_clock().now()

        # Compute absolute target position from relative or absolute goal
        try:
            self.absolute_target = self.resolve_target(goal)
        except ValueError as exc:
            self.reset_internal_state()
            goal_handle.abort()
            result = GoToResult()
            result.success = False
            result.message = str(exc)
            return result
        endpoint = self.absolute_target.copy()
        trajectory_start = self.current_position.copy()
        duration = getattr(goal, "duration", 0.0)
        if duration > 0.0:
            # Begin at the measured position, avoiding an endpoint jump before
            # the first trajectory update. Existing goals keep duration=0.
            self.absolute_target = trajectory_start.copy()
        if goal.relative:
            self.get_logger().info(
                f"Relative mode: current=({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}), "
                f"offset=({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f}), "
                f"absolute_target=({endpoint[0]:.2f}, {endpoint[1]:.2f}, {endpoint[2]:.2f})"
            )
        else:
            self.get_logger().info(
                f"Absolute mode: target=({endpoint[0]:.2f}, {endpoint[1]:.2f}, {endpoint[2]:.2f})"
            )

        rate = self.create_rate(20)

        while rclpy.ok():
            # Calculate time elapsed
            time_elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if goal_handle.is_cancel_requested:
                self.hold_current_position()
                goal_handle.canceled()
                result = GoToResult()
                result.success = False
                result.final_x, result.final_y, result.final_z = self.feedback_position
                result.message = "Goal canceled"
                self.get_logger().info("Goal canceled")
                self.reset_internal_state()
                self.destroy_rate(rate)
                return result

            if duration > 0.0:
                fraction = min(1.0, max(0.0, time_elapsed / duration))
                self.absolute_target = trajectory_start + fraction * (endpoint - trajectory_start)

            # Check the final endpoint, not the intermediate ramp position.
            distance_vector = endpoint - self.current_position
            distance_to_goal = np.linalg.norm(distance_vector)

            # Calculate individual axis errors
            x_error = abs(distance_vector[0])
            y_error = abs(distance_vector[1])
            z_error = abs(distance_vector[2])

            # Publish feedback
            feedback_msg = GoToFeedback()
            feedback_msg.current_x, feedback_msg.current_y, feedback_msg.current_z = (
                self.feedback_position
            )
            feedback_msg.distance_to_goal = float(distance_to_goal)
            publish_feedback_fn(feedback_msg)

            # Check if goal is reached
            if (
                time_elapsed >= duration
                and x_error <= goal.x_threshold
                and y_error <= goal.y_threshold
                and z_error <= goal.z_threshold
            ):
                goal_handle.succeed()
                result = GoToResult()
                result.success = True
                result.final_x, result.final_y, result.final_z = self.feedback_position
                result.time_elapsed = time_elapsed
                result.message = "Goal reached successfully"
                self.get_logger().info(
                    f"Goal reached! Final position: ({result.final_x:.2f}, {result.final_y:.2f}, {result.final_z:.2f})"
                )
                self.reset_internal_state()
                self.destroy_rate(rate)
                return result

            # Continue commanding position in control_loop_callback
            rate.sleep()

        self.destroy_rate(rate)
        return GoToResult()  # SHOULD NOT REACH HERE

    async def execute_goto_position_callback(self, goal_handle) -> GoToPosition.Result:
        """Execute GoToPosition action"""
        general_goal = GeneralGoal.from_position_goal(goal_handle.request)
        goto_result = await self.execute_callback(
            goal_handle,
            general_goal,
            publish_feedback_fn=lambda feedback: goal_handle.publish_feedback(
                GoToPosition.Feedback(feedback=feedback)
            ),
        )
        return GoToPosition.Result(result=goto_result)

    async def execute_takeoff_callback(self, goal_handle) -> Takeoff.Result:
        """Execute Takeoff action"""
        self.arm()
        self.engage_offboard_mode()
        general_goal = GeneralGoal.from_takeoff_goal(goal_handle.request)
        goto_result = await self.execute_callback(
            goal_handle,
            general_goal,
            publish_feedback_fn=lambda feedback: goal_handle.publish_feedback(
                Takeoff.Feedback(feedback=feedback)
            ),
        )
        return Takeoff.Result(result=goto_result)

    async def execute_land_callback(self, goal_handle) -> Land.Result:
        self.is_goal_active = True

        # check landed at a rate of 10hz
        rate = self.create_rate(10)
        start_time = self.get_clock().now()
        max_timeout = goal_handle.request.timeout
        max_timeout_out_of_land_mode = 5.0  # seconds
        is_in_land_mode = False
        time_elapsed_out_of_land_mode = 0.0
        start_time_out_of_land_mode = None

        self.engage_land_mode()
        while rclpy.ok():
            if (
                self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                and not is_in_land_mode
            ):
                self.get_logger().info("Entered land mode...")
                is_in_land_mode = True
                continue

            time_elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if time_elapsed > max_timeout:
                goal_handle.abort()
                result = GoToResult()
                result.success = False
                result.message = "Land action timed out"
                self.get_logger().info(
                    f"Land action timed out time elapsed: {time_elapsed: .3f} seconds"
                )
                self.destroy_rate(rate)
                self.reset_internal_state()
                return Land.Result(result=result)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = GoToResult()
                result.success = False
                result.message = "Land action canceled"
                self.get_logger().info("Land action canceled")
                self.reset_internal_state()
                self.destroy_rate(rate)
                return Land.Result(result=result)

            # to check if we exit land mode before landing is complete
            # 1. check started land mode previously
            # 2. check we are no longer in land mode
            # 3. check we are not disarmed yet
            # 4. check timeout exceeded
            if (
                is_in_land_mode
                and self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                and self.arming_state != VehicleStatus.ARMING_STATE_DISARMED
            ):
                if start_time_out_of_land_mode is None:  # first time out of land mode
                    start_time_out_of_land_mode = self.get_clock().now()

                # we start counting here if we exit land mode before disarm it means landing failed
                time_elapsed_out_of_land_mode = (
                    self.get_clock().now() - start_time_out_of_land_mode
                ).nanoseconds / 1e9

                if time_elapsed_out_of_land_mode <= max_timeout_out_of_land_mode:
                    continue

                goal_handle.abort()
                result = GoToResult()
                result.success = False
                result.message = f"Landing failed, exited land mode before disarm, exceeded timeout: {max_timeout_out_of_land_mode} seconds"
                self.get_logger().info(result.message)
                self.destroy_rate(rate)
                self.reset_internal_state()
                return Land.Result(result=result)

            feedback_msg = GoToFeedback()
            feedback_msg.current_x, feedback_msg.current_y, feedback_msg.current_z = (
                self.feedback_position
            )
            feedback_msg.distance_to_goal = -feedback_msg.current_z
            goal_handle.publish_feedback(Land.Feedback(feedback=feedback_msg))

            # rely on px4 land detector to disarm to indicate landed
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                goal_handle.succeed()
                result = GoToResult()
                result.success = True
                result.message = "Landed successfully"
                result.final_x, result.final_y, result.final_z = self.feedback_position
                result.time_elapsed = time_elapsed
                self.get_logger().info("Landed successfully")
                self.reset_internal_state()
                self.destroy_rate(rate)
                return Land.Result(result=result)

            rate.sleep()

        self.destroy_rate(rate)
        return Land.Result()  # SHOULD NOT REACH HERE

    def reset_internal_state(self):
        """Reset internal state variables"""
        self.absolute_target = None
        self.is_goal_active = False


def main(args=None):
    rclpy.init(args=args)
    offboard_node = OffboardNode()
    executor = MultiThreadedExecutor()
    executor.add_node(offboard_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        offboard_node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
