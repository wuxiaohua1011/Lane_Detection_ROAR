from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
import launch


def generate_launch_description():
    base_path = os.path.realpath(get_package_share_directory("lane_detection_indy"))
    video_path = Path(base_path) / "configs" / "P6010001.MOV"
    assert video_path.exists(), f"[{video_path}] does not exist"
    rviz_path = base_path + "/configs/lane_detection_3d_example.rviz"
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="rgb_camera_topic",
                default_value="/zed2i/zed_node/left/image_rect_color",
            ),
            launch.actions.DeclareLaunchArgument(
                name="ground_mask", default_value="[0,0,60,255,70,200]"
            ),
            launch.actions.DeclareLaunchArgument(
                name="grass_mask", default_value="[80,80,80,130,130,130]"
            ),
            launch.actions.DeclareLaunchArgument(
                name="horizon_pct", default_value="0.6"
            ),
            launch.actions.DeclareLaunchArgument(
                name="morph_kernel_size", default_value="1"
            ),
            launch.actions.DeclareLaunchArgument(name="debug", default_value="True"),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("lane_detection_indy"),
                        "lane_detection.launch.py",
                    )
                ),
                launch_arguments={
                    "rgb_camera_topic": launch.substitutions.LaunchConfiguration(
                        "rgb_camera_topic"
                    ),
                    "debug": launch.substitutions.LaunchConfiguration("debug"),
                    "morph_kernel_size": launch.substitutions.LaunchConfiguration(
                        "morph_kernel_size"
                    ),
                    "ground_mask": launch.substitutions.LaunchConfiguration(
                        "ground_mask"
                    ),
                    "grass_mask": launch.substitutions.LaunchConfiguration(
                        "grass_mask"
                    ),
                    "horizon_pct": launch.substitutions.LaunchConfiguration(
                        "horizon_pct"
                    ),
                }.items(),
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_path],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
