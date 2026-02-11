from setuptools import setup
from glob import glob
import os

package_name = "mppi"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="quddnr",
    maintainer_email="quddnr@todo.todo",
    description="Single-node MPPI offboard mission (takeoff->mppi->land) for PX4 via MAVROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mppi_node = mppi.mppi_node:main",
        ],
    },
)
