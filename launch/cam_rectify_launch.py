"""
Dynamically compose OpencvCamNode and ImageSubscriberNode in a component_container.

Limitations of this container:
 -- stdout is not set to flush after each line, so the most recent log messages may be delayed
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration,PythonExpression,TextSubstitution,PathJoinSubstitution



def generate_launch_description():

    cam_name_arg = DeclareLaunchArgument(
        name='cam_name', 
        default_value="cam0", 
        description="camera name") 
    
    cam_name = LaunchConfiguration('cam_name')

    cam_info = ComposableNode(
        package='opencv_cam',
        plugin='opencv_cam::OpencvCamNode',
        name= cam_name + '_info',
        parameters=[{
            'file': True,
            'filename': '',
            'camera_info_path': '/home/rborder/data/osprey/' + cam_name + '_intrinsics.yaml',
            'camera_frame_id': cam_name,
            'info_only': True,
            'fps': 20,
        }],
        remappings=[
            ('camera_info', '/alphasense_driver_ros/' + cam_name + '/camera_info'),
        ])
    
    cam_rectify = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name= cam_name + '_rectify',
        parameters=[{
            'output_width': 1440,
            'output_height': 1080,
        }],
        remappings=[
            ('image_raw', '/alphasense_driver_ros/' + cam_name + '/color/image'),
            ('camera_info', '/alphasense_driver_ros/' + cam_name + '/camera_info'),
            ('image_rect', '/alphasense_driver_ros/' + cam_name + '/color/image_rect'),
            ('camera_info_rect', '/alphasense_driver_ros/' + cam_name + '/camera_info_rect'),
        ])

    ld = LaunchDescription()
    ld.add_action(cam_name_arg)
    ld.add_action(cam_info)
    ld.add_action(cam_rectify)

    return ld
