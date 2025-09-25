from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_control', executable='motor_node', name='motor1'),
        Node(package='arm_control', executable='arm_node', name='arm'),
    ])

