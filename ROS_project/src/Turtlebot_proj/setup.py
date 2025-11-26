from setuptools import setup

package_name = "Turtlebot_proj"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/system.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="avirath",
    maintainer_email="you@example.com",
    description="TurtleBot3 ML + ROS2 navigation project",
    license="MIT",
    entry_points={
        "console_scripts": [
            "input_node = Turtlebot_proj.input_node:main",
            "decision_node = Turtlebot_proj.decision_node:main",
            "navigator_node = Turtlebot_proj.navigator_node:main",
        ],
    },
)
