import os
from glob import glob
from setuptools import find_packages, setup

package_name = "aruco_depth_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="michael",
    maintainer_email="michael@example.com",
    description="ROS 2 ArUco marker detection with depth camera",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_depth_node = aruco_depth_ros2.aruco_depth_node:main",
        ],
    },
)