"""Launch a rosrect-listener-agent"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='rosrect-listener-agent', node_executable='rosrect-listener-agent', output='screen'),
    ])