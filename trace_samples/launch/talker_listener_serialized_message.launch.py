import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    return launch.LaunchDescription([
        Trace(
            session_name='talker_listener_serialized_message',
            events_kernel=[''],
            events_ust=['ros2*']
        ),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='talker_serialized_message', output='screen'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='listener_serialized_message', output='screen'),
    ])
