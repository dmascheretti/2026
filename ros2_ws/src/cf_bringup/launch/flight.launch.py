"""Real flight: Crazyswarm2 server + our EKF, MPC, supervisor, mission.

    ros2 launch cf_bringup flight.launch.py scenario:=hover
    ros2 run cf_bringup kill_switch_node --ros-args -r __ns:=/cf231   (own terminal)
    ros2 service call /cf231/safety/arm std_srvs/srv/Trigger

Read docs/hardware_checklist.md before the first flight.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import control_nodes  # noqa: E402


def setup(context):
    name = LaunchConfiguration("name").perform(context)
    scenario = LaunchConfiguration("scenario").perform(context)
    share = get_package_share_directory("cf_bringup")
    scenario_file = os.path.join(share, "sim", "scenarios", scenario + ".yaml")
    crazyswarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("crazyflie"), "launch", "launch.py")),
        launch_arguments={
            "crazyflies_yaml_file": os.path.join(share, "config", "crazyflies.yaml"),
            "backend": "cpp",
            "mocap": "False",
            "teleop": "False",   # teleop would also publish on cmd_vel_legacy
            "rviz": "False",
        }.items())
    return [crazyswarm] + control_nodes(name, scenario_file, [0.0, 0.0, 0.0])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="cf231"),
        DeclareLaunchArgument("scenario", default_value="hover"),
        OpaqueFunction(function=setup),
    ])
