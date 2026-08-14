# 정적 장애 회피 스택 — path_follow_stanley_launch.py 와 동일 구성의 별칭.
#
#   ros2 launch path_following path_follow_static_avoid_launch.py
#   bash ~/f1tenth_ajou/scripts/run_control_node.sh
#
# 인자(track, enable_aeb, …)는 stanley 런치와 같다. 여기 노드를 더 넣지 말 것.

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    stanley = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "path_follow_stanley_launch.py",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(stanley)),
        ]
    )
