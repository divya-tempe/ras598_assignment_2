#!/usr/bin/env python3

"""
planner_node.py

Main ROS 2 planner node for the assignment.

Responsibilities:
1. Subscribe to /ground_truth and /energy_consumed
2. Call /get_task exactly once at startup
3. Load and prepare the map
4. Plan a path once using A*
5. Track the path using a simple waypoint-following controller
6. Publish /cmd_vel
7. Publish visualization markers on /planner_markers
8. Maintain a clean state-machine style flow

State machine:
- WAITING_FOR_TASK
- PLANNING
- WAITING_FOR_POSE
- TRACKING_PATH
- GOAL_REACHED
- FAILED
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

from ras598_assignment_2.task_client import TaskClient
from ras598_assignment_2.planner_core import PlannerCore
from ras598_assignment_2.controller import PathController
from ras598_assignment_2.marker_helper import build_marker_array


class PlannerNode(Node):
    """
    Main planner node.

    This node acts as the coordinator:
    - gets the task from /get_task
    - prepares the map and plans a path
    - waits for robot pose
    - tracks the path
    - publishes markers throughout execution
    """

    def __init__(self) -> None:
        """
        Initialize the ROS 2 node, helper modules, publishers, subscribers,
        timers, and internal state variables.
        """
        super().__init__('planner_node')

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        # Keep the map image path configurable for convenience during testing.
        self.declare_parameter('map_image_path', 'cave_filled.png')
        map_image_path = self.get_parameter('map_image_path').value

        # --------------------------------------------------------------
        # ROS publishers and subscribers
        # --------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/planner_markers', 10)

        self.ground_truth_sub = self.create_subscription(
            Odometry,
            '/ground_truth',
            self.ground_truth_callback,
            10
        )

        self.energy_sub = self.create_subscription(
            Float32,
            '/energy_consumed',
            self.energy_callback,
            10
        )

        # --------------------------------------------------------------
        # Helper modules
        # --------------------------------------------------------------
        self.task_client_helper = TaskClient(self, '/get_task')

        self.planner_core = PlannerCore(
            map_image_path=map_image_path,
            map_origin=(-8.0, -8.0),
            map_resolution=0.032,
            planning_resolution=0.2,
            inflation_radius_m=0.7
        )

        self.path_controller = PathController(
            linear_speed=1.6,
            angular_gain=1.8,
            max_linear_speed=1.6,
            max_angular_speed=2.0,
            waypoint_tolerance=0.3,
            goal_tolerance=0.2,
            heading_threshold=0.5,
            heading_deadband=0.05
        )

        # --------------------------------------------------------------
        # State machine values
        # --------------------------------------------------------------
        self.WAITING_FOR_TASK = 'WAITING_FOR_TASK'
        self.PLANNING = 'PLANNING'
        self.WAITING_FOR_POSE = 'WAITING_FOR_POSE'
        self.TRACKING_PATH = 'TRACKING_PATH'
        self.GOAL_REACHED = 'GOAL_REACHED'
        self.FAILED = 'FAILED'

        self.state = self.WAITING_FOR_TASK

        # --------------------------------------------------------------
        # Robot state
        # --------------------------------------------------------------
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_yaw = 0.0
        self.pose_received = False
        self.energy_consumed = 0.0

        # --------------------------------------------------------------
        # Task state
        # --------------------------------------------------------------
        self.start_world = None
        self.goal_world = None
        self.task_received = False
        self.task_request_sent = False

        # --------------------------------------------------------------
        # Planning state
        # --------------------------------------------------------------
        self.start_cell = None
        self.goal_cell = None

        self.raw_path_cells = []
        self.raw_path_world = []

        self.pruned_path_cells = []
        self.pruned_path_world = []

        self.map_ready = False
        self.path_ready = False
        self.planning_failed = False

        # --------------------------------------------------------------
        # Tracking state
        # --------------------------------------------------------------
        self.current_waypoint_index = 0
        self.goal_reached = False

        # --------------------------------------------------------------
        # Timers
        # --------------------------------------------------------------
        # Short startup timer used only to send /get_task once.
        self.startup_timer = self.create_timer(0.5, self.startup_step)

        # Main control loop timer.
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('planner_node started.')
        self.get_logger().info(f'Using map image: {map_image_path}')

    # ------------------------------------------------------------------
    # Startup and task acquisition
    # ------------------------------------------------------------------

    def startup_step(self) -> None:
        """
        Startup step that calls /get_task exactly once.

        This function is triggered by a timer so the node has time to start
        before trying to use the service.
        """
        if self.task_request_sent:
            return

        self.send_task_request()

    def send_task_request(self) -> None:
        """
        Wait for /get_task and send the Trigger request exactly once.
        """
        if self.task_request_sent:
            return

        if not self.task_client_helper.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('/get_task service not available yet...')
            return

        future = self.task_client_helper.send_request()

        if future is None:
            self.get_logger().warn('Could not send /get_task request.')
            return

        self.task_request_sent = True
        future.add_done_callback(self.handle_task_response)

        # We only want to call /get_task once.
        self.startup_timer.cancel()
        self.get_logger().info('Sent /get_task request.')

    def handle_task_response(self, future) -> None:
        """
        Handle the Trigger response from /get_task.

        The grading repo uses:
            response.message = "start_x,start_y,goal_x,goal_y"

        After parsing the result, the node moves to the PLANNING state.
        """
        try:
            response = future.result()

            if response is None:
                self.get_logger().error('Received empty response from /get_task.')
                self.state = self.FAILED
                self.planning_failed = True
                return

            if not response.success:
                self.get_logger().error(
                    f'/get_task returned success=False with message: {response.message}'
                )
                self.state = self.FAILED
                self.planning_failed = True
                return

            self.start_world, self.goal_world = self.task_client_helper.parse_task_message(
                response.message
            )

            self.task_received = True
            self.state = self.PLANNING

            self.get_logger().info(
                f'Task received. Start: {self.start_world}, Goal: {self.goal_world}'
            )

        except Exception as exc:
            self.get_logger().error(f'Failed to process /get_task response: {exc}')
            self.state = self.FAILED
            self.planning_failed = True

    # ------------------------------------------------------------------
    # ROS topic callbacks
    # ------------------------------------------------------------------

    def ground_truth_callback(self, msg: Odometry) -> None:
        """
        Store the latest robot pose from /ground_truth.

        /ground_truth is nav_msgs/msg/Odometry.
        """
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.pose_received = True

    def energy_callback(self, msg: Float32) -> None:
        """
        Store the latest energy value from /energy_consumed.

        /energy_consumed is std_msgs/msg/Float32.
        """
        self.energy_consumed = float(msg.data)

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def plan_mission(self) -> bool:
        """
        Prepare the map and plan the mission once.

        Planning steps:
        1. Prepare map
        2. Convert start and goal to grid cells
        3. Validate those cells
        4. Run A* on inflated grid
        5. Prune the raw path
        6. Convert both raw and pruned paths to world coordinates

        Returns:
            True if planning succeeded, False otherwise
        """
        try:
            # ----------------------------------------------------------
            # Prepare map only once
            # ----------------------------------------------------------
            if not self.planner_core.map_prepared:
                self.planner_core.prepare_map()
                self.map_ready = True
                self.get_logger().info('Map prepared successfully.')

            # ----------------------------------------------------------
            # Convert start and goal from world to planning grid
            # ----------------------------------------------------------
            self.start_cell = self.planner_core.world_to_grid(
                self.start_world[0],
                self.start_world[1]
            )

            self.goal_cell = self.planner_core.world_to_grid(
                self.goal_world[0],
                self.goal_world[1]
            )

            # ----------------------------------------------------------
            # Validate start and goal cells
            # ----------------------------------------------------------
            if not self.planner_core.is_in_bounds(self.start_cell, self.planner_core.inflated_grid):
                self.get_logger().error(f'Start cell out of bounds: {self.start_cell}')
                return False

            if not self.planner_core.is_in_bounds(self.goal_cell, self.planner_core.inflated_grid):
                self.get_logger().error(f'Goal cell out of bounds: {self.goal_cell}')
                return False

            if not self.planner_core.is_free(self.start_cell, self.planner_core.inflated_grid):
                self.get_logger().error(
                    f'Start cell is occupied after inflation: {self.start_cell}'
                )
                return False

            if not self.planner_core.is_free(self.goal_cell, self.planner_core.inflated_grid):
                self.get_logger().error(
                    f'Goal cell is occupied after inflation: {self.goal_cell}'
                )
                return False

            # ----------------------------------------------------------
            # A* planning on inflated grid
            # ----------------------------------------------------------
            self.raw_path_cells = self.planner_core.astar_search(
                self.start_cell,
                self.goal_cell,
                self.planner_core.inflated_grid
            )

            if len(self.raw_path_cells) == 0:
                self.get_logger().error('A* could not find a valid path.')
                return False

            self.raw_path_world = self.planner_core.cells_to_world_path(self.raw_path_cells)

            # ----------------------------------------------------------
            # Path pruning on inflated grid
            # ----------------------------------------------------------
            self.pruned_path_cells = self.planner_core.prune_path(
                self.raw_path_cells,
                self.planner_core.inflated_grid
            )

            if len(self.pruned_path_cells) == 0:
                self.get_logger().error('Path pruning failed.')
                return False

            self.pruned_path_world = self.planner_core.cells_to_world_path(
                self.pruned_path_cells
            )

            self.path_ready = True
            self.current_waypoint_index = 0

            self.get_logger().info(
                f'Planning complete. Raw path points: {len(self.raw_path_cells)}, '
                f'Pruned path points: {len(self.pruned_path_cells)}'
            )

            return True

        except Exception as exc:
            self.get_logger().error(f'Planning failed with exception: {exc}')
            return False

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def control_loop(self) -> None:
        """
        Main timer callback.

        This function keeps the node in a clear state-machine flow:
        - WAITING_FOR_TASK
        - PLANNING
        - WAITING_FOR_POSE
        - TRACKING_PATH
        - GOAL_REACHED
        - FAILED

        Markers are published continuously in every state.
        """
        # --------------------------------------------------------------
        # WAITING_FOR_TASK
        # --------------------------------------------------------------
        if self.state == self.WAITING_FOR_TASK:
            self.stop_robot()
            self.publish_visualization()
            return

        # --------------------------------------------------------------
        # PLANNING
        # --------------------------------------------------------------
        if self.state == self.PLANNING:
            planning_success = self.plan_mission()

            if planning_success:
                if self.pose_received:
                    self.state = self.TRACKING_PATH
                else:
                    self.state = self.WAITING_FOR_POSE
            else:
                self.planning_failed = True
                self.state = self.FAILED
                self.get_logger().error('Planner entered FAILED state.')

            self.stop_robot()
            self.publish_visualization()
            return

        # --------------------------------------------------------------
        # WAITING_FOR_POSE
        # --------------------------------------------------------------
        if self.state == self.WAITING_FOR_POSE:
            self.stop_robot()

            if self.pose_received:
                self.get_logger().info('Ground truth pose received. Starting path tracking.')
                self.state = self.TRACKING_PATH

            self.publish_visualization()
            return

        # --------------------------------------------------------------
        # FAILED
        # --------------------------------------------------------------
        if self.state == self.FAILED:
            self.stop_robot()
            self.publish_visualization()
            return

        # --------------------------------------------------------------
        # GOAL_REACHED
        # --------------------------------------------------------------
        if self.state == self.GOAL_REACHED:
            self.stop_robot()
            self.publish_visualization()
            return

        # --------------------------------------------------------------
        # TRACKING_PATH
        # --------------------------------------------------------------
        if self.state == self.TRACKING_PATH:
            # Safety checks
            if not self.pose_received or not self.path_ready or len(self.pruned_path_world) == 0:
                self.stop_robot()
                self.publish_visualization()
                return

            # Advance waypoint if needed
            self.advance_waypoint_if_needed()

            # If final goal was reached during waypoint advancement
            if self.goal_reached:
                self.state = self.GOAL_REACHED
                self.stop_robot()
                self.publish_visualization()
                return

            # Get the current target waypoint
            # lookahead = 2
            # target_index = min(
            #     self.current_waypoint_index + lookahead,
            #     len(self.pruned_path_world) - 1
            # )
            current_target_world = self.pruned_path_world[self.current_waypoint_index]

            
            if current_target_world is None:
                self.goal_reached = True
                self.state = self.GOAL_REACHED
                self.stop_robot()
                self.publish_visualization()
                return

            # Compute and publish control command
            cmd = self.path_controller.compute_cmd(
                self.current_pose_x,
                self.current_pose_y,
                self.current_yaw,
                current_target_world[0],
                current_target_world[1]
            )

            self.cmd_vel_pub.publish(cmd)
            self.publish_visualization()
            return

        # --------------------------------------------------------------
        # Unknown state fallback
        # --------------------------------------------------------------
        self.get_logger().warn(f'Unknown state: {self.state}')
        self.stop_robot()
        self.publish_visualization()

    # ------------------------------------------------------------------
    # Tracking helpers
    # ------------------------------------------------------------------

    def advance_waypoint_if_needed(self) -> None:
        """
        Advance to the next waypoint when the current one is reached.

        Also checks if the final goal has been reached.
        """
        if self.goal_world is None or len(self.pruned_path_world) == 0:
            return

        # First check final goal condition.
        if self.path_controller.goal_reached(
            self.current_pose_x,
            self.current_pose_y,
            self.goal_world[0],
            self.goal_world[1]
        ):
            self.goal_reached = True
            return

        # If current waypoint index is already past the path, treat as done.
        if self.current_waypoint_index >= len(self.pruned_path_world):
            self.goal_reached = True
            return

        current_target = self.pruned_path_world[self.current_waypoint_index]

        if self.path_controller.waypoint_reached(
            self.current_pose_x,
            self.current_pose_y,
            current_target[0],
            current_target[1]
        ):
            self.current_waypoint_index += 1

            # If we advanced past the last waypoint, mark as complete.
            if self.current_waypoint_index >= len(self.pruned_path_world):
                self.goal_reached = True

    def get_current_target_waypoint(self):
        """
        Return the current target waypoint in world coordinates.

        Returns:
            (x, y) target waypoint, or None if no waypoint is available
        """
        if not self.path_ready:
            return None

        if self.current_waypoint_index < 0:
            return None

        if self.current_waypoint_index >= len(self.pruned_path_world):
            return None

        return self.pruned_path_world[self.current_waypoint_index]

    def stop_robot(self) -> None:
        """
        Publish a zero Twist command to stop the robot.
        """
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def publish_visualization(self) -> None:
        """
        Build and publish the planner MarkerArray.

        Markers are published continuously so RViz always shows the latest:
        - start
        - goal
        - raw path
        - pruned path
        - current target waypoint
        - robot position
        """
        current_target_world = self.get_current_target_waypoint()

        robot_world = None
        if self.pose_received:
            robot_world = (self.current_pose_x, self.current_pose_y)

        marker_array = build_marker_array(
            start_world=self.start_world,
            goal_world=self.goal_world,
            raw_path_world=self.raw_path_world,
            pruned_path_world=self.pruned_path_world,
            current_target_world=current_target_world,
            robot_world=robot_world,
            frame_id='map'
        )

        self.marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def quaternion_to_yaw(self, orientation) -> float:
        """
        Convert quaternion orientation to yaw angle.

        Args:
            orientation: geometry_msgs/Quaternion

        Returns:
            yaw angle in radians
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None) -> None:
    """
    Standard ROS 2 Python entry point.
    """
    rclpy.init(args=args)
    node = PlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('planner_node interrupted by user.')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()