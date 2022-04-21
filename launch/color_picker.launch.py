from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
import launch


def generate_launch_description():
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="rgb_camera_topic", default_value="/rgb_image"
            ),
            launch_ros.actions.Node(
                package="lane_detection_indy",
                executable="color_picker_node",
                name="color_picker_node",
                parameters=[
                    {
                        "rgb_camera_topic": launch.substitutions.LaunchConfiguration(
                            "rgb_camera_topic"
                        ),
                    }
                ],
            ),
        ]
    )
