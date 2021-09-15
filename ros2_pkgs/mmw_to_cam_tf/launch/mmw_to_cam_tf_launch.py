from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
						#tf2_ros
    	Node(  package = "tf2_ros", 
               executable = "static_transform_publisher",
               arguments = ["0", "0", "0", "0", "0", "0", "camera", "mmwave_pl_frame"]
            ),

	])
