"""Nodes shared by the simulation and the flight launch files."""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def control_nodes(name, scenario_file, initial_position):
    """EKF, MPC, safety supervisor and mission, in namespace /<name>."""
    share = get_package_share_directory("cf_bringup")
    params_file = os.path.join(share, "model", "params.yaml")
    ekf_config = os.path.join(get_package_share_directory("cf_ekf"), "config", "ekf.yaml")
    safety_config = os.path.join(share, "config", "safety.yaml")
    return [
        Node(package="cf_ekf", executable="ekf_node", namespace=name, output="screen",
             parameters=[{"params_file": params_file, "ekf_config_file": ekf_config,
                          "initial_position": initial_position}]),
        Node(package="cf_mpc", executable="mpc_node", namespace=name, output="screen",
             parameters=[{"params_file": params_file}]),
        Node(package="cf_bringup", executable="safety_supervisor_node", namespace=name,
             output="screen", parameters=[{"safety_config_file": safety_config}]),
        Node(package="cf_bringup", executable="mission_node", namespace=name, output="screen",
             parameters=[{"scenario_file": scenario_file}]),
    ]
