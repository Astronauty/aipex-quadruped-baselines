from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Resolve the params file from the installed package share directory so the
    # launch file works on any machine. CMakeLists.txt installs config/ to
    # share/convex_mpc/config, so this requires the package to have been built.
    convex_mpc_params_file = os.path.join(
        get_package_share_directory('convex_mpc'),
        'config',
        'convex_mpc_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='convex_mpc',
            executable='convex_mpc_controller',
            name='convex_mpc_controller',
            parameters = [convex_mpc_params_file]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',   
        )


    ])