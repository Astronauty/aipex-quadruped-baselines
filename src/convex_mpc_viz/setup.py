import os
from glob import glob
from setuptools import setup

package_name = 'convex_mpc_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Nguyen',
    maintainer_email='jezemba@andrew.cmu.edu',
    description="RViz visualisation for the convex MPC gait planner.",
    license='MIT',
    entry_points={
        'console_scripts': [
            'swing_plan_markers = convex_mpc_viz.swing_plan_markers:main',
        ],
    },
)
