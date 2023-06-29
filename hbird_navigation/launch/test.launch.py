from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hbird_navigation',
            namespace='turtlesim1',
            executable='ground_control_node',
            name='ground_control'
        ),
        Node(
            package='hbird_navigation',
            namespace='HB1',
            executable='agent_control_node',
            name='HB1_agent'
        ),
        Node(
            package='hbird_navigation',
            namespace='HB2',
            executable='agent_control_node',
            name='HB2_agent'
        ),
         Node(
            package='hbird_navigation',
            namespace='HB1',
            executable='crazyflie_viz_node',
            name='HB1_crazyflie'
        ),
        Node(
            package='hbird_navigation',
            namespace='HB2',
            executable='crazyflie_viz_node',
            name='HB2_crazyflie'
        )
    ])


# Figuring out launch files: https://roboticsbackend.com/ros2-launch-file-example/