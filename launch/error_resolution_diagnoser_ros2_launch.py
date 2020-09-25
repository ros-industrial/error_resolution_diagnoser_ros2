"""Launch a error_resolution_diagnoser_ros2"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='error_resolution_diagnoser_ros2', node_executable='error_resolution_diagnoser_ros2', output='screen', arguments=['__log_disable_rosout:=true']),
    ])
