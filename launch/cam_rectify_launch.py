
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

    cam_info = ComposableNode(
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
        }],
        remappings=[
            ('image_raw', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image')]),
            ('camera_info', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/camera_info')]),
        ])
    
    cam_info_container = ComposableNodeContainer(
        name=[cam_name, TextSubstitution(text= '_info_container')],
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            cam_info,
        ],
        output='screen',
        )
    
    cam_decompress = Node(
        package='image_transport',
        executable='republish',
        name=[cam_name, TextSubstitution(text= '_decompress')],
        parameters=[{
            'use_sim_time': True,
        }],
        namespace='',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image/compressed')]),
            ('out', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image')])]
    )
    
    cam_rectify = ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            parameters=[{
                'use_sim_time': True,
            }],
            remappings=[
                ('image', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image')]),
                ('camera_info', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/camera_info')]),
                ('image_rect', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image_rect')])
            ],
        )
    
    cam_rectify_container = ComposableNodeContainer(
        namespace='',
        name=[cam_name, TextSubstitution(text='_rectify_container')],
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[cam_rectify],
    )

    # cam_rectify = ComposableNode(
    #     package='isaac_ros_image_proc',
    #     plugin='nvidia::isaac_ros::image_proc::RectifyNode',
    #     name= [cam_name, TextSubstitution(text='_rectify')],
    #     parameters=[{
    #         'output_width': 1440,
    #         'output_height': 1080,
    #     }],
    #     remappings=[
    #         ('image_raw',[TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image')]),
    #         ('camera_info', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/camera_info')]),
    #         ('image_rect', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/color/image_rect')]),
    #         ('camera_info_rect', [TextSubstitution(text='/alphasense_driver_ros/'), cam_name, TextSubstitution(text='/camera_info_rect')]),
    #     ])

    # cam_rectify_container = ComposableNodeContainer(
    #     package='rclcpp_components',
    #     name=[cam_name, TextSubstitution(text='_rectify_container')],
    #     namespace='',
    #     executable='component_container_mt',
    #     composable_node_descriptions=[
    #         cam_rectify,
    #     ],
    #     output='screen'
    #     )

    ld = LaunchDescription()
    ld.add_action(cam_name_arg)
    ld.add_action(cam_decompress)
    ld.add_action(cam_info_container)
    ld.add_action(cam_rectify_container)

    return ld
