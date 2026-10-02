from glob import glob
import os

from setuptools import find_packages, setup


package_name = "drone_cslam"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="quddnr",
    maintainer_email="quddnr@todo.todo",
    description="Offline inter-UAV LiDAR registration and pose-graph experiments.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gazebo_gt_recorder = drone_cslam.gt_recorder_node:main",
            "oracle_drift_eval = drone_cslam.oracle_eval:main",
            "lidar_relative_eval = drone_cslam.lidar_eval:main",
        ],
    },
)
