import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    use_rosbag = launch.substitutions.LaunchConfiguration('use_rosbag', default='false')
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"),
        launch.actions.DeclareLaunchArgument("use_rosbag", default_value="false"),
        Trace(
            session_name='end_to_end_sample',
            events_kernel=[],
            events_ust=['ros2*']
        ),
        launch_ros.actions.Node(
            package='caret_demos', executable='end_to_end_sample', output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'use_rosbag': use_rosbag}]),
    ])
