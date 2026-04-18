#!/usr/bin/env python3

"""
marker_helper.py

This file contains helper functions for creating RViz visualization markers.

The goal is to make the planner output easy to understand by showing:
- start point
- goal point
- raw path
- pruned path
- current target waypoint
- optional robot position

The functions are kept simple and reusable so planner_node.py can call them
without cluttering its own logic.
"""

from typing import List, Optional, Tuple

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


# ------------------------------------------------------------------
# Marker ID constants
# ------------------------------------------------------------------
START_MARKER_ID = 0
GOAL_MARKER_ID = 1
RAW_PATH_MARKER_ID = 2
PRUNED_PATH_MARKER_ID = 3
TARGET_MARKER_ID = 4
ROBOT_MARKER_ID = 5


# ------------------------------------------------------------------
# Basic helper functions
# ------------------------------------------------------------------
def make_point(x: float, y: float, z: float = 0.0) -> Point:
    """
    Create a geometry_msgs/Point.

    Args:
        x: x coordinate
        y: y coordinate
        z: z coordinate

    Returns:
        Point message
    """
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def make_base_marker(
    marker_id: int,
    namespace: str,
    frame_id: str = 'map'
) -> Marker:
    """
    Create a base Marker with common fields filled in.

    Args:
        marker_id: unique marker ID inside its namespace
        namespace: marker namespace
        frame_id: RViz frame ID

    Returns:
        Marker with common fields initialized
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


# ------------------------------------------------------------------
# Individual marker builders
# ------------------------------------------------------------------
def make_start_marker(
    start_xy: Tuple[float, float],
    frame_id: str = 'map'
) -> Marker:
    """
    Create a marker for the start point.

    Args:
        start_xy: (x, y) of start point
        frame_id: RViz frame ID

    Returns:
        Marker for the start location
    """
    marker = make_base_marker(START_MARKER_ID, 'planner_start', frame_id)
    marker.type = Marker.SPHERE

    marker.pose.position.x = float(start_xy[0])
    marker.pose.position.y = float(start_xy[1])
    marker.pose.position.z = 0.15

    marker.scale.x = 0.35
    marker.scale.y = 0.35
    marker.scale.z = 0.35

    # Green for start
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    return marker


def make_goal_marker(
    goal_xy: Tuple[float, float],
    frame_id: str = 'map'
) -> Marker:
    """
    Create a marker for the goal point.

    Args:
        goal_xy: (x, y) of goal point
        frame_id: RViz frame ID

    Returns:
        Marker for the goal location
    """
    marker = make_base_marker(GOAL_MARKER_ID, 'planner_goal', frame_id)
    marker.type = Marker.SPHERE

    marker.pose.position.x = float(goal_xy[0])
    marker.pose.position.y = float(goal_xy[1])
    marker.pose.position.z = 0.15

    marker.scale.x = 0.35
    marker.scale.y = 0.35
    marker.scale.z = 0.35

    # Red for goal
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    return marker


def make_raw_path_marker(
    path_world: List[Tuple[float, float]],
    frame_id: str = 'map'
) -> Marker:
    """
    Create a LINE_STRIP marker for the raw A* path.

    Args:
        path_world: list of (x, y) world coordinates
        frame_id: RViz frame ID

    Returns:
        Marker representing the raw path
    """
    marker = make_base_marker(RAW_PATH_MARKER_ID, 'planner_raw_path', frame_id)
    marker.type = Marker.LINE_STRIP

    marker.scale.x = 0.08

    # Yellow for raw path
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.4

    for x, y in path_world:
        marker.points.append(make_point(x, y, 0.05))

    return marker


def make_pruned_path_marker(
    path_world: List[Tuple[float, float]],
    frame_id: str = 'map'
) -> Marker:
    """
    Create a LINE_STRIP marker for the final pruned path.

    Args:
        path_world: list of (x, y) world coordinates
        frame_id: RViz frame ID

    Returns:
        Marker representing the pruned path
    """
    marker = make_base_marker(PRUNED_PATH_MARKER_ID, 'planner_pruned_path', frame_id)
    marker.type = Marker.LINE_STRIP

    marker.scale.x = 0.12

    # Blue for final path
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.4
    marker.color.b = 1.0

    for x, y in path_world:
        marker.points.append(make_point(x, y, 0.10))

    return marker


def make_target_marker(
    target_xy: Tuple[float, float],
    frame_id: str = 'map'
) -> Marker:
    """
    Create a marker for the current target waypoint.

    Args:
        target_xy: (x, y) of current target waypoint
        frame_id: RViz frame ID

    Returns:
        Marker representing the current target waypoint
    """
    marker = make_base_marker(TARGET_MARKER_ID, 'planner_target', frame_id)
    marker.type = Marker.CYLINDER

    marker.pose.position.x = float(target_xy[0])
    marker.pose.position.y = float(target_xy[1])
    marker.pose.position.z = 0.2

    marker.scale.x = 0.30
    marker.scale.y = 0.30
    marker.scale.z = 0.40

    # Magenta for current target
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0

    return marker


def make_robot_marker(
    robot_xy: Tuple[float, float],
    frame_id: str = 'map'
) -> Marker:
    """
    Create a marker for the robot's current position.

    Args:
        robot_xy: (x, y) robot position
        frame_id: RViz frame ID

    Returns:
        Marker representing the robot position
    """
    marker = make_base_marker(ROBOT_MARKER_ID, 'planner_robot', frame_id)
    marker.type = Marker.CUBE

    marker.pose.position.x = float(robot_xy[0])
    marker.pose.position.y = float(robot_xy[1])
    marker.pose.position.z = 0.15

    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25

    # White for robot position
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    return marker


# ------------------------------------------------------------------
# Marker array builder
# ------------------------------------------------------------------
def build_marker_array(
    start_world: Optional[Tuple[float, float]],
    goal_world: Optional[Tuple[float, float]],
    raw_path_world: Optional[List[Tuple[float, float]]],
    pruned_path_world: Optional[List[Tuple[float, float]]],
    current_target_world: Optional[Tuple[float, float]],
    robot_world: Optional[Tuple[float, float]],
    frame_id: str = 'map'
) -> MarkerArray:
    """
    Build a MarkerArray containing all available planner visualization items.

    Any item that is None or empty is skipped.

    Args:
        start_world: (x, y) start position or None
        goal_world: (x, y) goal position or None
        raw_path_world: raw path as world points or None
        pruned_path_world: pruned path as world points or None
        current_target_world: active waypoint or None
        robot_world: robot position or None
        frame_id: RViz frame ID

    Returns:
        MarkerArray with all currently valid markers
    """
    marker_array = MarkerArray()

    if start_world is not None:
        marker_array.markers.append(
            make_start_marker(start_world, frame_id)
        )

    if goal_world is not None:
        marker_array.markers.append(
            make_goal_marker(goal_world, frame_id)
        )

    if raw_path_world is not None and len(raw_path_world) > 0:
        marker_array.markers.append(
            make_raw_path_marker(raw_path_world, frame_id)
        )

    if pruned_path_world is not None and len(pruned_path_world) > 0:
        marker_array.markers.append(
            make_pruned_path_marker(pruned_path_world, frame_id)
        )

    if current_target_world is not None:
        marker_array.markers.append(
            make_target_marker(current_target_world, frame_id)
        )

    if robot_world is not None:
        marker_array.markers.append(
            make_robot_marker(robot_world, frame_id)
        )

    return marker_array