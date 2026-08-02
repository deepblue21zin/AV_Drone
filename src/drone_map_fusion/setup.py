from setuptools import find_packages, setup

package_name = "drone_map_fusion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="quddnr",
    maintainer_email="quddnr@todo.todo",
    description="Known-pose occupancy grid fusion for multi-UAV mapping.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_fusion_node = drone_map_fusion.map_fusion_node:main",
        ],
    },
)
