import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    return launch.LaunchDescription([
        Trace(
            session_name='multi_talker_listener',
            events_kernel=[],
            events_ust=['ros2*']
        ),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='talker', output='screen', namespace='ns1'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='talker', output='screen', namespace='ns2'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='listener', output='screen', namespace='ns1'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='listener', output='screen', namespace='ns2'),
    ])
