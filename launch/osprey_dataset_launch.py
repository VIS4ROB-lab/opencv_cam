"""
Dynamically compose OpencvCamNode and ImageSubscriberNode in a component_container.

Limitations of this container:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import Node, ComposableNode
from launch.substitutions import LaunchConfiguration,PythonExpression,TextSubstitution,PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    opencv_cam_path = get_package_share_directory('opencv_cam')

    cam0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(opencv_cam_path + '/launch/cam_rectify_launch.py'),
        launch_arguments = {
            'cam_name': "cam0",
        }.items()
    )
    cam1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(opencv_cam_path + '/launch/cam_rectify_launch.py'),
        launch_arguments = {
            'cam_name': "cam1",
        }.items()
    )
    cam2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(opencv_cam_path + '/launch/cam_rectify_launch.py'),
        launch_arguments = {
            'cam_name': "cam2",
        }.items()
    )

    play_bag = Node(
        package='rosbag2_transport',
        executable='player',
        name='player',
        output="screen",
        parameters=["/home/rborder/data/osprey/osprey_play.yaml"],
    )

    ld = LaunchDescription()
    ld.add_action(cam0)
    ld.add_action(cam1)
    ld.add_action(cam2)
    ld.add_action(play_bag)

    return ld
