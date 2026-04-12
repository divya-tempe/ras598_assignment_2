#!/usr/bin/env python3

"""
task_client.py

Helper class for calling the /get_task service.

The service type is example_interfaces/srv/Trigger.
The service response stores the task inside response.message
using the format:

    "start_x,start_y,goal_x,goal_y"

Example:
    "-7.0,-7.0,7.0,2.5"

This helper keeps the service logic separate from planner_node.py
so the main node stays easier to read and explain.
"""

from rclpy.node import Node
from example_interfaces.srv import Trigger


class TaskClient:
    """
    Simple helper for requesting the start and goal from /get_task.
    """

    def __init__(self, node: Node, service_name: str = '/get_task') -> None:
        """
        Create the service client.

        Args:
            node: ROS 2 node that owns this client
            service_name: name of the service to call
        """
        self.node = node
        self.service_name = service_name
        self.client = self.node.create_client(Trigger, self.service_name)
        self.request = Trigger.Request()
        self.request_sent = False

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        """
        Wait for the /get_task service to become available.

        Args:
            timeout_sec: maximum time to wait in seconds

        Returns:
            True if the service is available, False otherwise
        """
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def send_request(self):
        """
        Send an asynchronous Trigger request to /get_task.

        Returns:
            A Future object from call_async(), or None if request was not sent
        """
        if not self.client.service_is_ready():
            self.node.get_logger().warn(
                f'Service {self.service_name} is not ready yet.'
            )
            return None

        self.request_sent = True
        future = self.client.call_async(self.request)
        return future

    def parse_task_message(self, message: str) -> tuple[tuple[float, float], tuple[float, float]]:
        """
        Parse the task string from response.message.

        Expected format:
            "start_x,start_y,goal_x,goal_y"

        Args:
            message: task string returned by the Trigger service

        Returns:
            start_world: (start_x, start_y)
            goal_world: (goal_x, goal_y)

        Raises:
            ValueError: if the message is empty, badly formatted,
                        or cannot be converted to floats
        """
        if not message:
            raise ValueError('Task message is empty.')

        parts = [part.strip() for part in message.split(',')]

        if len(parts) != 4:
            raise ValueError(
                f'Expected 4 comma-separated values, got {len(parts)}: "{message}"'
            )

        try:
            start_x = float(parts[0])
            start_y = float(parts[1])
            goal_x = float(parts[2])
            goal_y = float(parts[3])
        except ValueError as exc:
            raise ValueError(
                f'Could not convert task message to floats: "{message}"'
            ) from exc

        start_world = (start_x, start_y)
        goal_world = (goal_x, goal_y)
        return start_world, goal_world