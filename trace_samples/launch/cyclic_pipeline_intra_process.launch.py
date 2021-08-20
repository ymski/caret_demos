import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    return launch.LaunchDescription([
        Trace(
            session_name='cyclic_pipeline_intra_process',
            events_kernel=[]
        ),
        launch_ros.actions.Node(
            package='intra_process_demo',
            executable='cyclic_pipeline',
            output='screen'),
    ])
