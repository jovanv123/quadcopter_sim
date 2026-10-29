from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():


    pkg_share = get_package_share_directory('drone_mission')
    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models'))

    package_share = FindPackageShare("drone_mission")
    world_file = PathJoinSubstitution(
        [package_share, "worlds", "multicopter_velocity_control.sdf"]
    )
    bridge_config = PathJoinSubstitution(
        [package_share, "config", "bridge.yaml"]
    )
    mission_config = PathJoinSubstitution(
        [package_share, "config", "mission.yaml"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    mission = Node(
        package="drone_mission",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[mission_config, {"use_sim_time": True}],
    )

    perception = Node(
        package = "drone_perception",
        executable ="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

        return LaunchDescription([set_model_path, gazebo, bridge, mission, perception])