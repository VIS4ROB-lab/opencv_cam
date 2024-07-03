
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration,PythonExpression,TextSubstitution,PathJoinSubstitution

def generate_launch_description():

    cam_name_arg = DeclareLaunchArgument(
        name='cam_name', 
        default_value="cam0", 
        description="camera name") 
    
    cam_name = LaunchConfiguration('cam_name')

    cam_decompress = ComposableNode(
        namespace='',
        package='image_transport',
        plugin='image_transport::Republisher',
        name=[cam_name, TextSubstitution(text= '_decompress')],
        parameters=[{
            'in_transport': 'compressed',
            'out_transport': 'raw',
            'use_sim_time': True,
        }],
        remappings=[
            ('in/compressed', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image/compressed')]),
            ('out', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image')])],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    cam_info = ComposableNode(
        namespace='',
        package='opencv_cam',
        plugin='opencv_cam::OpencvCamNode',
        name= [cam_name, TextSubstitution(text= '_info')],
        parameters=[{
            'file': True,
            'filename': '',
            'camera_info_path': [TextSubstitution(text= '/home/rborder/data/osprey/'), cam_name, TextSubstitution(text= '_intrinsics.yaml')],
            'camera_frame_id': [cam_name, TextSubstitution(text= '_sensor_frame')],
            'info_only': True,
            'fps': 20,
            'width': 1440,
            'use_sim_time': True,
            'in_transport': 'raw',
        }],
        remappings=[
            ('image_raw', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image')]),
            ('camera_info', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/camera_info')]),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    
    cam_rectify = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name=[cam_name, TextSubstitution(text='_rectify')],
            parameters=[{
                'use_sim_time': True,
            }],
            remappings=[
                ('image', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image')]),
                ('camera_info', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/camera_info')]),
                ('image_rect', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image_rect')])
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    cam_container = ComposableNodeContainer(
        namespace='',
        package='rclcpp_components',
        name=[cam_name, TextSubstitution(text='_rectify_container')],
        executable='component_container_mt',
        composable_node_descriptions=[
            cam_decompress,
            cam_info,
            cam_rectify,
        ],
        output='screen'
        )

    ld = LaunchDescription()
    ld.add_action(cam_name_arg)
    ld.add_action(cam_container)

    return ld
