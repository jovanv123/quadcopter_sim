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

    tracker = Node(
        package="drone_mission",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    base_to_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.1', '--y', '0', '--z', '-0.04',
                '--roll', '0', '--pitch', '0.9', '--yaw', '0',
                '--frame-id', 'x3/base_footprint', '--child-frame-id', 'camera_link'],
        parameters=[{'use_sim_time': True}],
    )

    camera_to_optical = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'camera_link', '--child-frame-id', 'camera_color_optical_frame'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([set_model_path, gazebo, bridge, base_to_camera, camera_to_optical,
                          mission, perception, tracker])