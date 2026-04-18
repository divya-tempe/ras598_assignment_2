#!/usr/bin/env python3

"""
controller.py

This file contains a simple waypoint-following controller.

Main idea:
1. Look at the current target waypoint
2. Compute distance and heading error to that waypoint
3. If the robot is not facing the waypoint well enough, rotate first
4. If the robot is roughly aligned, move forward while correcting heading
5. Stop when the waypoint or final goal is reached

The controller is designed to be:
- simple
- robust
- easy to explain in a viva
- easy to tune during testing
"""

import math
from geometry_msgs.msg import Twist


class PathController:
    """
    Simple waypoint-following controller.

    The controller uses:
    - waypoint tolerance for intermediate waypoints
    - goal tolerance for the final goal
    - rotate-first behavior when heading error is large
    - bounded linear and angular speeds
    """

    def __init__(
        self,
        linear_speed: float = 1.4,
        angular_gain: float = 2.0,
        max_linear_speed: float = 1.4,
        max_angular_speed: float = 1.5,
        waypoint_tolerance: float = 0.6,
        goal_tolerance: float = 0.3,
        heading_threshold: float = 0.45,
        heading_deadband: float = 0.05
    ) -> None:
        """
        Initialize controller parameters.

        Args:
            linear_speed: nominal forward speed when aligned
            angular_gain: proportional gain for heading correction
            max_linear_speed: upper bound on forward speed
            max_angular_speed: upper bound on turning speed
            waypoint_tolerance: distance threshold for intermediate waypoints
            goal_tolerance: distance threshold for final goal
            heading_threshold: if abs(heading error) is above this,
                               rotate first before moving forward
            heading_deadband: small heading errors below this are treated
                              as zero to reduce jitter
        """
        self.linear_speed = linear_speed
        self.angular_gain = angular_gain
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.waypoint_tolerance = waypoint_tolerance
        self.goal_tolerance = goal_tolerance
        self.heading_threshold = heading_threshold
        self.heading_deadband = heading_deadband

    # ------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------

    def normalize_angle(self, angle: float) -> float:
        """
        Wrap an angle to the range [-pi, pi].

        Args:
            angle: input angle in radians

        Returns:
            normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        """
        Clamp a value between min_value and max_value.

        Args:
            value: input value
            min_value: lower limit
            max_value: upper limit

        Returns:
            clamped value
        """
        return max(min_value, min(value, max_value))

    def distance_to_point(
        self,
        current_x: float,
        current_y: float,
        target_x: float,
        target_y: float
    ) -> float:
        """
        Compute Euclidean distance from robot position to target point.

        Args:
            current_x: robot x position
            current_y: robot y position
            target_x: target x position
            target_y: target y position

        Returns:
            distance in meters
        """
        dx = target_x - current_x
        dy = target_y - current_y
        return math.sqrt(dx * dx + dy * dy)

    def heading_to_point(
        self,
        current_x: float,
        current_y: float,
        target_x: float,
        target_y: float
    ) -> float:
        """
        Compute desired heading angle from current position to target point.

        Args:
            current_x: robot x position
            current_y: robot y position
            target_x: target x position
            target_y: target y position

        Returns:
            desired heading angle in radians
        """
        return math.atan2(target_y - current_y, target_x - current_x)

    def waypoint_reached(
        self,
        current_x: float,
        current_y: float,
        target_x: float,
        target_y: float
    ) -> bool:
        """
        Check whether an intermediate waypoint has been reached.

        Args:
            current_x: robot x position
            current_y: robot y position
            target_x: waypoint x position
            target_y: waypoint y position

        Returns:
            True if the waypoint is within waypoint_tolerance
        """
        distance = self.distance_to_point(current_x, current_y, target_x, target_y)
        return distance <= self.waypoint_tolerance

    def goal_reached(
        self,
        current_x: float,
        current_y: float,
        goal_x: float,
        goal_y: float
    ) -> bool:
        """
        Check whether the final goal has been reached.

        Args:
            current_x: robot x position
            current_y: robot y position
            goal_x: goal x position
            goal_y: goal y position

        Returns:
            True if the goal is within goal_tolerance
        """
        distance = self.distance_to_point(current_x, current_y, goal_x, goal_y)
        return distance <= self.goal_tolerance

    def make_stop_cmd(self) -> Twist:
        """
        Create a zero-velocity Twist command.

        Returns:
            Twist with all velocities set to zero
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        return cmd

    # ------------------------------------------------------------------
    # Main control logic
    # ------------------------------------------------------------------

    def compute_cmd(
        self,
        current_x: float,
        current_y: float,
        current_yaw: float,
        target_x: float,
        target_y: float,
        use_goal_tolerance: bool = False
    ) -> Twist:
        """
        Compute a Twist command to move toward the current waypoint.

        If use_goal_tolerance is True, use goal_tolerance for stopping;
        otherwise, use waypoint_tolerance.
        """
        cmd = Twist()

        distance_error = self.distance_to_point(
            current_x, current_y, target_x, target_y
        )
        desired_heading = self.heading_to_point(
            current_x, current_y, target_x, target_y
        )
        heading_error = self.normalize_angle(desired_heading - current_yaw)
        if abs(heading_error) < self.heading_deadband:
            heading_error = 0.0

        tolerance = self.goal_tolerance if use_goal_tolerance else self.waypoint_tolerance
        if distance_error <= tolerance:
            return self.make_stop_cmd()

        angular_z = self.angular_gain * heading_error
        angular_z = self.clamp(
            angular_z,
            -self.max_angular_speed,
            self.max_angular_speed
        )
        if abs(heading_error) > self.heading_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = angular_z
            return cmd

        heading_scale = max(0.2, 1.0 - abs(heading_error) / self.heading_threshold)
        distance_scale = min(1.0, distance_error / 1.0)
        linear_x = self.linear_speed * heading_scale * distance_scale
        linear_x = self.clamp(linear_x, 0.0, self.max_linear_speed)
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        return cmd
