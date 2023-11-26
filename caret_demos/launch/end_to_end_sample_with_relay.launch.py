import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='caret_demos', executable='end_to_end_sample', output='screen'),
        launch_ros.actions.Node(
            package='caret_demos', executable='relay', output='screen'),
    ])
