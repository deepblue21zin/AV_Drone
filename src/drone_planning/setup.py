from setuptools import find_packages, setup

package_name = "drone_planning"

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
    description="Planning foundation package for sensor-aware single and multi-drone autonomy.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
    "console_scripts": [
        "local_planner_node = drone_planning.local_planner_node:main",
        "local_planner_node_sjee_fix = drone_planning.local_planner_node_sjee_fix:main",
        "astar_global_planner = drone_planning.astar_global_planner_node:main",
    ],
},
)
