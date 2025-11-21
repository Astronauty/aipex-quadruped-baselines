from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # convex_mpc_params_file = os.path.join(
    #     get_package_share_directory('convex_mpc'),
    #     'config',
    #     'convex_mpc_params.yaml'
    # )

    # convex_mpc_params_file = os.path.join(
    #     os.path.dirname(__file__),  
    #     "..",
    #     "config",
    #     "convex_mpc_params.yaml"
    # )
    # convex_mpc_params_file = "/home/aipexws5/daniel/aipex-quadruped-baselines/src/convex_mpc/config/convex_mpc_params.yaml"
    convex_mpc_params_file = "/home/abien/ros2_ws/src/aipex-quadruped-baselines/src/convex_mpc/config/convex_mpc_params.yaml"
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