from setuptools import find_packages, setup

package_name = "global_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/global_planner.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aqua Autonomy",
    maintainer_email="user@example.com",
    description="Global planner ROS2 node: /fused_buoys + /odom -> TaskMaster -> path + cmd_vel",
    license="MIT",
    entry_points={
        "console_scripts": [
            "global_planner_node = global_planner.global_planner_node:main",
            "task1_cheese_node = global_planner.task1_cheese_node:main",
        ],
    },
)
