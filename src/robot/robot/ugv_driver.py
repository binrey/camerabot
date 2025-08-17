import os
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Twist

from .ugv_base_ctrl import BaseController

# Keep the same kinematic parameters as the Webots driver by default
HALF_DISTANCE_BETWEEN_WHEELS = 0.045


class UGVDriver:
    def __init__(self) -> None:
        self.__node = None  # type: ignore[assignment]
        self.__base = None  # type: ignore[assignment]
        self.__target_twist = Twist()
        self.__half_wheel_separation = HALF_DISTANCE_BETWEEN_WHEELS
        self.__max_linear_speed = 1.3

    def init(self, robot_node: Any, properties: Optional[Dict[str, Any]] = None) -> None:
        # robot_node is accepted for interface compatibility with Webots but is unused here
        del robot_node

        # Parameters (overridable via properties)
        properties = properties or {}
        self.__half_wheel_separation = float(
            properties.get("half_wheel_separation", HALF_DISTANCE_BETWEEN_WHEELS)
        )
        self.__max_linear_speed = float(properties.get("max_linear_speed", 1.3))

        # Serial connection parameters (overridable via properties or env)
        uart_dev = properties.get("uart_device") or os.environ.get("UGV_UART", "/dev/ttyAMA0")
        baud_rate = int(properties.get("baud_rate") or os.environ.get("UGV_BAUD", 115200))

        # Hardware controller
        self.__base = BaseController(uart_dev, baud_rate)

        # ROS 2 node and subscription
        rclpy.init(args=None)
        self.__node = rclpy.create_node("ugv_driver")
        self.__node.create_subscription(Twist, "cmd_vel", self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist: Twist) -> None:
        self.__target_twist = twist

    def step(self) -> None:
        # Process a single ROS 2 cycle
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Convert Twist to differential wheel linear speeds (m/s)
        forward_speed = float(self.__target_twist.linear.x)
        angular_speed = float(self.__target_twist.angular.z)

        left_linear_speed = forward_speed - angular_speed * self.__half_wheel_separation
        right_linear_speed = forward_speed + angular_speed * self.__half_wheel_separation

        # Scale to controller input range [-255, 255] using configured max linear speed
        scale = 255.0 / self.__max_linear_speed if self.__max_linear_speed > 0 else 0.0
        left_cmd = int(max(-255, min(255, left_linear_speed * scale)))
        right_cmd = int(max(-255, min(255, right_linear_speed * scale)))

        # Send to base
        self.__base.base_speed_ctrl(left_cmd, right_cmd)


def main(args: Optional[list[str]] = None) -> None:
    del args
    driver = UGVDriver()
    driver.init(robot_node=None, properties=None)
    try:
        while True:
            driver.step()
            time.sleep(0.02)  # 50 Hz
    except KeyboardInterrupt:
        pass


