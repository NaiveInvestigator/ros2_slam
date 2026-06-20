"""
Python port of orbslam3_ros2.launch.py

Updated to launch orb_slam_rgbd.py (the Python node) instead of a compiled
C++ executable.  All other behaviour (bag recording, RViz2, OctoMap) is
preserved identically.

Launch arguments:
  camera_type   mono | rgbd | stereo          (default: mono)
  visualize     true | false                  (default: false)
  record_bag    true | false                  (default: false)
  start_octomap true | false                  (default: false)

Example:
  ros2 launch orbslam3_ros2 orbslam3_ros2.launch.py camera_type:=rgbd visualize:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# Conditional RViz2 launch  (identical logic to the original)
# ---------------------------------------------------------------------------

def launch_rviz2(context):
    visualize = context.launch_configurations.get("visualize", "false")
    if visualize.lower() == "true":
        package_share_directory = get_package_share_directory("orbulation")
        rviz_config_path = os.path.join(
            package_share_directory, "config", "orbslam3_rviz2.rviz"
        )
        return [
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config_path],
                output="screen",
            )
        ]
    return []


# ---------------------------------------------------------------------------
# Main launch description
# ---------------------------------------------------------------------------

def generate_launch_description():

    slam_pkg_path = get_package_share_directory("orbulation")

    vocab_file    = os.path.join(slam_pkg_path, "config", "ORBvoc.txt")
    settings_file = os.path.join(slam_pkg_path, "config", "TUM_RGB-D_Dataset.yaml")

    print(f"Path of vocab file    : {vocab_file}")
    print(f"Path of settings file : {settings_file}")

    # ---- Launch arguments (unchanged from original) ------------------------
    record_bag_arg = DeclareLaunchArgument(
        "record_bag",
        default_value="false",
        description="Enable or disable ros2 bag recording",
    )
    visualize_arg = DeclareLaunchArgument(
        "visualize",
        default_value="false",
        description="Launch RViz2 with saved configuration",
    )
    start_octomap_arg = DeclareLaunchArgument(
        "start_octomap",
        default_value="false",
        description="Start Octomap server",
    )
    camera_type_arg = DeclareLaunchArgument(
        "camera_type",
        default_value="mono",
        description="Camera type: mono, rgbd, stereo",
    )
    show_viewer_arg = DeclareLaunchArgument(
        "show_viewer",
        default_value="false",
        description="Show the ORB-SLAM3 Pangolin viewer (requires a display / GTK)",
    )

    # ---- SLAM node (Python script instead of C++ executable) ---------------
    #
    # The original C++ launch used:
    #   executable=LaunchConfiguration('camera_type')
    # to pick between 'mono', 'rgbd', 'stereo' executables at runtime.
    #
    # The Python equivalent runs orb_slam_rgbd.py directly as a ROS2 node.
    # Swap the script path below when mono / stereo Python ports are added.
    #
    slam_node = Node(
        package="orbulation",
        executable="orb_slam_rgbd",   # entry-point registered in setup.py
        name="orbslam3_rgbd_node",
        output="screen",
        parameters=[
            {"vocab_path":   vocab_file},
            {"config_path":  settings_file},
            {"show_viewer":  LaunchConfiguration("show_viewer")},
        ],
    )

    # ---- ros2 bag record (identical to original) ---------------------------
    bag_record_process = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a"],
        condition=IfCondition(LaunchConfiguration("record_bag")),
        output="screen",
    )

    # ---- OctoMap server (identical to original) ----------------------------
    octomap_server_node = ExecuteProcess(
        cmd=[
            "ros2", "run", "octomap_server", "octomap_server_node",
            "--ros-args", "--remap", "cloud_in:=/slam/pointcloud",
        ],
        condition=IfCondition(LaunchConfiguration("start_octomap")),
    )

    return LaunchDescription([
        record_bag_arg,
        start_octomap_arg,
        visualize_arg,
        camera_type_arg,
        show_viewer_arg,
        slam_node,
        bag_record_process,
        OpaqueFunction(function=launch_rviz2),
        octomap_server_node,
    ])
