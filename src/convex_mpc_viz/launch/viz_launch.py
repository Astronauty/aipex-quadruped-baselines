"""RViz visualisation for the convex MPC gait planner.

Brings up, alongside a controller that is already running:
  - robot_state_publisher, posing the Go2 URDF from the controller's
    /joint_states
  - the swing-plan marker node
  - RViz with a preset configuration

The controller itself publishes TF world->base and /joint_states, so this
launch file adds only visualisation and can be started and stopped freely
without touching the control loop.

Mesh paths: the shipped URDF refers to meshes as package://go2_description/...,
but that package is not on the ament path (it lives under /opt/unitree_robotics
without an index entry). Rather than vendor 25 MB of .dae into this repo, the
URDF is read and its mesh URIs are rewritten to absolute file:// paths at launch
time. Override the location with the go2_mesh_dir argument if the install moves.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MESH_DIR = '/opt/unitree_robotics/unitree_ros/robots/go2_description/dae'


def _load_urdf(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf').perform(context)
    mesh_dir = LaunchConfiguration('go2_mesh_dir').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context).lower() in ('1', 'true', 'yes')

    with open(urdf_path, 'r') as f:
        urdf = f.read()

    # Rewrite package:// mesh URIs to absolute paths that RViz can open.
    if os.path.isdir(mesh_dir):
        urdf = urdf.replace('package://go2_description/dae/', f'file://{mesh_dir}/')
        urdf = urdf.replace('package://go2_description/meshes/', f'file://{mesh_dir}/')
    else:
        print(f'[viz_launch] WARNING: mesh dir not found: {mesh_dir}\n'
              f'[viz_launch] The URDF will load but render as untextured boxes.')

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': urdf,
                # The controller broadcasts world->base itself, from the measured
                # body pose, so robot_state_publisher must not also publish a
                # root transform or the two will fight.
                'publish_frequency': 50.0,
            }],
        ),
        Node(
            package='convex_mpc_viz',
            executable='swing_plan_markers',
            name='swing_plan_markers',
            output='screen',
            parameters=[{
                'frame_id': 'world',
                'show_future_swings': True,
                'show_both_foot_sources': True,
            }],
        ),
    ]

    if use_rviz:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ))

    return nodes


def generate_launch_description():
    default_urdf = os.path.join(
        get_package_share_directory('convex_mpc'), 'urdf', 'go2_description.urdf')
    default_rviz = os.path.join(
        get_package_share_directory('convex_mpc_viz'), 'rviz', 'convex_mpc.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('urdf', default_value=default_urdf,
                              description='Go2 URDF to display.'),
        DeclareLaunchArgument('go2_mesh_dir', default_value=DEFAULT_MESH_DIR,
                              description='Directory holding the Go2 .dae meshes.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz,
                              description='RViz config to load.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Set false to run the markers without RViz.'),
        OpaqueFunction(function=_load_urdf),
    ])
