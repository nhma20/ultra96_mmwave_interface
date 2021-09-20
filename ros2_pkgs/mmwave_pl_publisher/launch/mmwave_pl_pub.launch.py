from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mmwave_pl_publisher',
            executable='mmwave_publisher_node',
            output="screen",
            parameters=[
            			{"port": "/dev/ttyUSB1"},
            			{"baud": 115200},
            			{"n_points": 8}
            ]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0","0","0","0","0","0","camera", "mmwave_frame"],
            output="screen")

    ])
