#!/usr/bin/env python3

import asyncio
import threading
from concurrent.futures import CancelledError, Future, TimeoutError
from typing import Dict, Optional

import rclpy
from bb_uav_msgs.action import Actuation
from mavsdk import System
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ActuatorControlNode(Node):
    ON = 1
    OFF = -1

    def __init__(self):
        super().__init__("actuator_control_node")

        self.address = (
            self.declare_parameter(
                "px4_address",
                value="udpin://0.0.0:14540",
            )
            .get_parameter_value()
            .string_value
        )

        self.actuation_server_name = (
            self.declare_parameter(
                "actuation_server_name",
                value="/uav/tins/actuation",
            )
            .get_parameter_value()
            .string_value
        )

        self.actuation_indexes = (
            self.declare_parameter(
                "actuation_indexes",
                value=[1],
            )
            .get_parameter_value()
            .integer_array_value
        )

        self.timeout = (
            self.declare_parameter(
                "timeout",
                value=5.0,
            )
            .get_parameter_value()
            .double_value
        )

        # ROS Action Server
        self.actuation_action_server = ActionServer(
            self,
            Actuation,
            self.actuation_server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # MAVSDK
        self.drone = System()
        self.connection_future: Optional[Future] = None
        # self.is_connected = False

        # Asyncio infrastructure
        self.async_loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(
            target=self._run_async_loop,
            daemon=True,
        )
        self.async_thread.start()

        self.running_tasks: Dict[str, Future] = {}

        self.get_logger().info("Actuator control node started")

    # --------------------------- MAVSDK async tasks ---------------------------

    def _run_async_loop(self):
        asyncio.set_event_loop(self.async_loop)
        self.get_logger().info("Async event loop thread started")
        self.async_loop.run_forever()

    def _run_async(self, coro) -> Future:
        return asyncio.run_coroutine_threadsafe(coro, self.async_loop)

    async def _connect(self):
        # if self.is_connected:
        #     return True

        self.get_logger().info(f"Connecting to PX4 at {self.address}...")
        await self.drone.connect(system_address=self.address)
        self.get_logger().info("Connected to PX4")
        return True

    async def _check_connection(self):
        """Continuously watch the connection state until trued"""
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                return True
            self.get_logger().info("Connection state: not connected")

    async def _actuate(self, enable: bool):
        tasks = [
            self.drone.action.set_actuator(
                idx,
                self.ON if enable else self.OFF,
            )
            for idx in self.actuation_indexes
        ]

        await asyncio.gather(*tasks)

    # --------------------------- ROS Action callbacks ---------------------------

    async def goal_callback(self, goal_request):
        self.get_logger().info("Received actuation goal")

        # self.connection_future = self._run_async(self._connect())

        # self.get_logger().info("Waiting for PX4 connection...")
        # self.get_logger().info(f"PX4 connection established: {self.connection_future.result(), self.connection_future.done()}")

        # self.is_connected = self._run_async(self._check_connection()).result(timeout=self.timeout)

        # self.get_logger().info(f"PX4 is connected: {self.is_connected}")
        return GoalResponse.ACCEPT

    async def cancel_callback(self, goal_handle):
        goal_id = str(goal_handle.goal_id)

        self.get_logger().info("Cancelling goal...")

        future = self.running_tasks.get(goal_id)
        if future and not future.done():
            future.cancel()
            del self.running_tasks[goal_id]

        if self.connection_future and not self.connection_future.done():
            self.connection_future.cancel()

        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal_id = str(goal_handle.goal_id)
        result = Actuation.Result(success=False)

        try:
            self.connection_future = self._run_async(self._connect())

            self.get_logger().info("Waiting for PX4 connection...")
            self.get_logger().info(
                f"PX4 connection established: {self.connection_future.result(timeout=self.timeout), self.connection_future.done()}"
            )

            is_connected = self._run_async(self._check_connection()).result(
                timeout=self.timeout
            )
            self.get_logger().info(f"PX4 is connected: {is_connected}")

            if not is_connected:
                result.message = "PX4 not connected"
                goal_handle.abort()
                return result

            future = self._run_async(
                self._actuate(goal_handle.request.enable_actuation)
            )
            self.running_tasks[goal_id] = future

            future.result(timeout=self.timeout)

            result.success = True
            result.message = "Actuation successful"
            goal_handle.succeed()
            return result

        except (asyncio.CancelledError, CancelledError):
            self.get_logger().error("Actuation cancelled")
            result.message = "Actuation cancelled"
            goal_handle.canceled()
            return result

        except TimeoutError:  # this is from concurrent.futures
            self.get_logger().error("Actuation timed out")
            result.message = "Actuation timed out"
            goal_handle.abort()
            return result

        except Exception as e:
            self.get_logger().error(f"Actuation failed: {e}")
            import traceback

            traceback.print_exc()
            result.message = f"Error: {e}"
            goal_handle.abort()
            return result

        finally:
            self.running_tasks.pop(goal_id, None)
            self.connection_future = None
            # self.is_connected = False # TODO: see if want to not force the connect everytime

    def destroy_node(self):
        self.get_logger().info("Shutting down actuator control node")

        self.async_loop.call_soon_threadsafe(self.async_loop.stop)
        self.get_logger().info("Waiting for async event loop thread to finish...")
        self.async_thread.join()

        while self.async_loop.is_running():
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ActuatorControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
