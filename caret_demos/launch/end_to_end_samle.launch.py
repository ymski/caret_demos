import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    return launch.LaunchDescription([
        Trace(
            session_name='end_to_end_sample',
            events_kernel=[],
            events_ust=['ros2*']
        ),
        launch_ros.actions.Node(
            package='trace_samples', executable='end_to_end_sample', output='screen'),
    ])
