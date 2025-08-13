import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def load_yaml_file(yaml_file_path):
    try:
        with open(yaml_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as err:
        print(f"Could not load YAML file. Error: {err}")
        return None

def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path', default= \
        os.path.join(get_package_share_directory("fast_lio_localization_sc_qn"), "config"))
    config_path_value = config_path.perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_use = LaunchConfiguration('rviz', default='true')

    default_rviz_config_path = os.path.join(
        config_path_value, 'localization_rviz.rviz')

    rviz_cfg = LaunchConfiguration('rviz_cfg', default=default_rviz_config_path)
    params_file = os.path.join(
        config_path_value,
        "config.yaml"
    )

    fast_lio_localization_node = Node(
        package="fast_lio_localization_sc_qn",
        executable="fast_lio_localization_sc_qn_node",
        name="fast_lio_localization_sc_qn_node",
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    return [
        fast_lio_localization_node,
        rviz_node
    ]

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
