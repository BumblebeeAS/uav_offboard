import os
from glob import glob

from setuptools import find_packages, setup

package_name = "uav_offboard"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@todo.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "offboard_node = uav_offboard.offboard_node:main",
            "landing_target_pose_node = uav_offboard.landing_target_pose_node:main",
            "imu_repub = uav_offboard.imu_repub:main",
            "actuator_control_node = uav_offboard.actuator_control_node:main",
        ],
    },
)
