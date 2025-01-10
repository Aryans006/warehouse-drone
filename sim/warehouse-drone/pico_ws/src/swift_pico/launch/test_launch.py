import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('rotors_swift_gazebo')
    pkg_project_gazebo = get_package_share_directory('swift_pico')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Setup to launch the simulator and Gazebo world

    pico_server = Node(
        package='swift_pico',
        executable='pico_server.py',
        name='pico_server'

    )

    waypoint_service = Node(
        package='swift_pico',
        executable='waypoint_service.py',
        name='waypoint_service'

    )
    pico_cilent = Node(
        package='swift_pico',
        executable='pico_client.py',
        name='pico_client'

    )
    path_planning_service = Node(
        package='swift_pico',
        executable='path_planning_service.py',
        name='path_planning_service'

    )

    rosbag = launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'task_2b', '/whycon/poses','/random_points'],
            output='screen'
        )

    return LaunchDescription([
        # gz_sim,
        # bridge,
        # roll_pitch_yawrate_thrust_controller,
        # swift_interface,
        # whycon,
        # image_view,
        # planner_server,
        waypoint_service,
        pico_server, 
        # path_planning_service,
        pico_cilent
        # rosbag
    ])