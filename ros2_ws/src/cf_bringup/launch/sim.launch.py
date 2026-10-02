"""Full ROS pipeline against the simulated Crazyflie (no radio).

    ros2 launch cf_bringup sim.launch.py scenario:=steps
    ros2 service call /cf1/safety/arm std_srvs/srv/Trigger
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import control_nodes  # noqa: E402


def setup(context):
    name = LaunchConfiguration("name").perform(context)
    scenario = LaunchConfiguration("scenario").perform(context)
    quantize_flow = LaunchConfiguration("quantize_flow").perform(context) == "true"
    share = get_package_share_directory("cf_bringup")
    scenario_file = os.path.join(share, "sim", "scenarios", scenario + ".yaml")
    initial_position = [0.0, 0.0, 0.0]
    sim_bridge = Node(
        package="cf_bringup", executable="sim_bridge_node", namespace=name, output="screen",
        parameters=[{"params_file": os.path.join(share, "model", "params.yaml"),
                     "sim_config_file": os.path.join(share, "sim", "sim.yaml"),
                     "initial_position": initial_position,
                     "quantize_flow": quantize_flow}])
    return [sim_bridge] + control_nodes(name, scenario_file, initial_position)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("name", default_value="cf1"),
        DeclareLaunchArgument("scenario", default_value="hover"),
        # Round flow to integer counts like the real sensor (see docs).
        DeclareLaunchArgument("quantize_flow", default_value="true"),
        OpaqueFunction(function=setup),
    ])
