from setuptools import setup
import os
from glob import glob

package_name = 'path_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
    ],
    install_requires=['setuptools', 'numpy', 'pyserial'],
    zip_safe=True,
    maintainer='woong',
    maintainer_email='user@example.com',
    description='F1TENTH 실차 경로 추종 및 회피 노드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fgm_node = path_following.fgm_node:main',
            'local_planner_node = path_following.local_planner_node:main',
            'static_obstacle_node = path_following.static_obstacle_node:main',
            'integrated_obstacle_node = path_following.integrated_obstacle_node:main',
            'drive_strategy_node = path_following.drive_strategy_node:main',
            'stanley_waypoint_follow_node = path_following.stanley_waypoint_follow_node:main',
            'control_node = path_following.control_node:main',
            'csv_logger_node = path_following.csv_logger_node:main',
            'vehicle_measurement_node = path_following.vehicle_measurement_node:main',
            'telemetry_dummy_publisher = path_following.telemetry_dummy_publisher:main',
            'drive_monitor = path_following.drive_monitor:main',
            'stack_status_node = path_following.stack_status_node:main',
            'straight_drive_publisher = path_following.straight_drive_publisher:main',
        ],
    },
)
