
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration,PythonExpression,TextSubstitution,PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    opencv_cam_path = get_package_share_directory('opencv_cam')

    cam_container = ComposableNodeContainer(
        namespace='',
        package='rclcpp_components',
        name='cam_container',
        executable='component_container_mt',
        output='screen'
    )

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

    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--clock', '400', '/home/rborder/data/osprey/industrial_building_a/ind_a_autonomy_mission_1_flight_1/ind_a_autonomy_mission_1_flight_1'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(cam_container)
    ld.add_action(cam0)
    ld.add_action(cam1)
    ld.add_action(cam2)
    ld.add_action(play_bag)

    return ld
