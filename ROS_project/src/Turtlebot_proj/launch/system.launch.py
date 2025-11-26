from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="Turtlebot_proj",
            executable="input_node",
            output="screen"
        ),
        Node(
            package="Turtlebot_proj",
            executable="decision_node",
            output="screen"
        ),
        Node(
            package="Turtlebot_proj",
            executable="navigator_node",
            output="screen"
        ),
    ])
