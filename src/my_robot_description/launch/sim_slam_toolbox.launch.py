"""
sim_slam_toolbox.launch.py
==========================
Launch file for SLAM Toolbox (Asynchronous) simulation on the 11th Floor world.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # ── Package paths ──────────────────────────────────────────────────
    pkg_share = get_package_share_directory("my_robot_description")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    # ── File paths ─────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_share, "urdf", "diff_drive_robot.urdf.xacro")
    world_file = os.path.join(pkg_share, "worlds", "eleventh_floor.world")
    rviz_config = os.path.join(pkg_share, "rviz", "hector_slam.rviz")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")

    # ── Process xacro → URDF string ───────────────────────────────────
    robot_description_raw = xacro.process_file(urdf_file).toxml()

    # ── Launch arguments ──────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    # ── 1. Gazebo (server + client) ───────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_file}.items(),
    )

    # ── 2. Robot State Publisher ──────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_raw,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ── 3. Spawn Robot at (0, 0, 0) ──────────────────────────────────
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_entity",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "diff_drive_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
        ],
    )

    # ── 4. SLAM Toolbox (Async) ───────────────────────────────────────
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── 5. RViz2 ─────────────────────────────────────────────────────
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── Build launch description ──────────────────────────────────────
    return LaunchDescription(
        [
            declare_use_sim_time,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            slam_toolbox,
            rviz2,
        ]
    )
