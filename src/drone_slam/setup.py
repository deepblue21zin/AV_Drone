from setuptools import find_packages, setup

package_name = "drone_slam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="quddnr",
    maintainer_email="quddnr@todo.todo",
    description="SLAM scaffold package for separated mapping/localization development.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "slam_scaffold_node = drone_slam.slam_scaffold_node:main",
            "simple_2d_mapping_node = drone_slam.simple_2d_mapping_node:main",
        ],
    },
)
