#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    assignment_share_dir = get_package_share_directory('ras598_assignment_2')

    map_yaml_path = os.path.join(assignment_share_dir, 'map.yaml')
    cave_image_path = os.path.join(assignment_share_dir, 'cave_filled.png')

    grading_scout_path = os.path.join(assignment_share_dir, 'grading_scout.py')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    grading_scout_node = ExecuteProcess(
        cmd=['python3', grading_scout_path],
        output='screen'
    )

    planner_node = Node(
        package='ras598_assignment_2',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{'map_image_path': cave_image_path}],
    )

    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('stage_ros2'),
                'launch',
                'stage.launch.py'
            ])
        ),
        launch_arguments={'world': 'cave'}.items(),
    )

    rviz_config_path = '/ras598_assignment_2/planning.rviz'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        stage_launch,
        planner_node,
        rviz_node,
        map_server_node,
        lifecycle_manager_node,
        grading_scout_node
    ])








# #!/usr/bin/env python3

# """
# planner_launch.py

# ROS 2 launch file for Assignment 2.

# This launch file does four things:
# 1. Starts nav2_map_server with the provided map.yaml
# 2. Starts the lifecycle manager so map_server becomes active
# 3. Starts grading_scout.py from the assignment repo
# 4. Starts the student's planner node from the ras598_assignment_2 package

# This version uses standard ROS 2 launch style with LaunchDescription,
# which is cleaner and more typical than manually using LaunchService in a
# Python main() function.
# """

# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     """
#     Build and return the full launch description.

#     The repo behavior is preserved:
#     - nav2_map_server is launched
#     - lifecycle manager is launched for map_server
#     - grading_scout.py is launched

#     In addition, the student's planner node is launched from the
#     ras598_assignment_2 package.
#     """

#     # --------------------------------------------------------------
#     # Find the assignment package share directory
#     #
#     # We use this to locate map.yaml inside the provided repo package.
#     # --------------------------------------------------------------
#     assignment_share_dir = get_package_share_directory('ras598_assignment_2')
#     map_yaml_path = os.path.join(assignment_share_dir, 'map.yaml')

#     # --------------------------------------------------------------
#     # Map server node
#     #
#     # This publishes the map using the provided map.yaml.
#     # --------------------------------------------------------------
#     map_server_node = Node(
#         package='nav2_map_server',
#         executable='map_server',
#         name='map_server',
#         output='screen',
#         parameters=[
#             {'yaml_filename': map_yaml_path}
#         ]
#     )

#     # --------------------------------------------------------------
#     # Lifecycle manager
#     #
#     # nav2_map_server is a lifecycle node, so it needs a lifecycle
#     # manager to configure and activate it automatically.
#     # --------------------------------------------------------------
#     lifecycle_manager_node = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_mapper',
#         output='screen',
#         parameters=[
#             {'autostart': True},
#             {'node_names': ['map_server']}
#         ]
#     )

#     # --------------------------------------------------------------
#     # Grading scout node
#     #
#     # This preserves the repo's grading behavior by running the
#     # provided grading_scout.py script.
#     # --------------------------------------------------------------
#     grading_scout_node = Node(
#         package='ras598_assignment_2',
#         executable='grading_scout.py',
#         name='grading_scout',
#         output='screen'
#     )

#     # --------------------------------------------------------------
#     # Student planner node
#     #
#     # This launches your ROS 2 Python planner node from the package.
#     # The map image path is passed as a parameter so planner_node.py
#     # can load cave_filled.png correctly.
#     # --------------------------------------------------------------
#     planner_node = Node(
#         package='ras598_assignment_2',
#         executable='planner_node',
#         name='planner_node',
#         output='screen',
#         parameters=[
#             {
#                 'map_image_path': os.path.join(assignment_share_dir, 'cave_filled.png')
#             }
#         ]
#     )

#     # --------------------------------------------------------------
#     # Return all nodes together in one launch description
#     # --------------------------------------------------------------
#     return LaunchDescription([
#         map_server_node,
#         lifecycle_manager_node,
#         grading_scout_node,
#         planner_node
#     ])