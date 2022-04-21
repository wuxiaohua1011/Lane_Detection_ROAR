from setuptools import setup
import os
from glob import glob
from urllib import request

package_name = "lane_detection_indy"

setup(
    name=package_name,
    version="0.0.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (
            os.path.join(os.path.join("share", package_name), "configs"),
            glob("configs/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="roar",
    maintainer_email="ztl1998@berkeley.edu",
    description="Lane Detection Package from ROAR Indy",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rgb_streamer = lane_detection_indy.test_pub:main",
            "lane_detect = lane_detection_indy.lane_detect:main",
            "flood_fill = lane_detection_indy.floodfill:main",
            "rg_detect = lane_detection_indy.rg_detect:main",
            "merge_mask = lane_detection_indy.combine_mask:main",
            "video_player_node = lane_detection_indy.video_player_node:main",
            "road_mask_node = lane_detection_indy.road_mask_node:main",
            "color_picker_node = lane_detection_indy.color_picker_node:main",
        ],
    },
)
