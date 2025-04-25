import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():

    ld = LaunchDescription()
    package_share_dir = get_package_share_directory('nedo_bringup')
    default_front_calib_path = os.path.join(package_share_dir, 'config', 'front_camera_ost.yaml')
    default_left_calib_path = os.path.join(package_share_dir, 'config', 'left_camera_ost.yaml')
    default_right_calib_path = os.path.join(package_share_dir, 'config', 'right_camera_ost.yaml')
    #default_calib_path = os.path.join(get_package_share_directory('nedo_bringup'),'config','ost.yaml')

    # Launch引数として各カメラのキャリブレーションファイルを設定
    front_calib_file = LaunchConfiguration('front_calib_file')
    left_calib_file = LaunchConfiguration('left_calib_file')
    right_calib_file = LaunchConfiguration('right_calib_file')
    #calib_file =LaunchConfiguration('calib_file')
    #arg_calib_file = DeclareLaunchArgument('calib_file',
    #                                       default_value='file://'+default_calib_path,
    #                                       description='calibration yaml file')
    arg_front_calib_file = DeclareLaunchArgument(
        'front_calib_file',
        default_value='file://' + default_front_calib_path,
        description='Front camera calibration YAML file'
    )
    arg_left_calib_file = DeclareLaunchArgument(
        'left_calib_file',
        default_value='file://' + default_left_calib_path,
        description='Left camera calibration YAML file'
    )
    arg_right_calib_file = DeclareLaunchArgument(
        'right_calib_file',
        default_value='file://' + default_right_calib_path,
        description='Right camera calibration YAML file'
    )
    declare_lidar_type_cmd = DeclareLaunchArgument('lidar_type',
                                                   default_value='livox',
                                                   description='type of lidar (ex:livox or pandar)')
    
    topic_ns = 'nedo'

    container_name = 'data_collection'
    #hesai_launch_path = get_package_share_directory('hesai_ros_driver')
    livox_launch_path = get_package_share_directory('livox_ros2_avia')

    #hesai_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([hesai_launch_path,'/launch/start.py']),
    #    condition = LaunchConfigurationEquals('lidar_type','pandar')
    #    )
    
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([livox_launch_path,'/launch/livox_lidar_launch.py']),
        condition = LaunchConfigurationEquals('lidar_type','livox')
    )
    
    container = Node(
        name= container_name,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        )
    #load_composable_nodes = LoadComposableNodes(
    #    target_container=container_name,
    #    composable_node_descriptions=[
    #        ComposableNode(
    #            package='image_transport',
    #            plugin='image_transport::SubscriberFilter',
    #            name='image_subscriber',
    #            namespace=topic_ns,
    #            parameters=[
    #                {'image_transport': 'raw'}
    #            ],
    #            remappings=[
    #                ('image', '/zed2i/zed_node/left_raw/image_raw_color')
    #            ]
    #        ),
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            #ComposableNode(
            #    package='nedo_capture',
            #    plugin='nedo_capture::NedoCapNode',
            #    name='calib_cap',
            #    namespace=topic_ns
            #),

            # フロントカメラ映像取得
            ComposableNode(
            package='nedo_capture',
            plugin='nedo_capture::NedoCapNode',
            name='front_cap',
            namespace=topic_ns + '/front',
            parameters=[
                {'device': '/dev/front_camera'},
                {'frame_id': 'front_camera_frame'}
            ]
            ),
            # 左カメラ映像取得
            ComposableNode(
            package='nedo_capture',
            plugin='nedo_capture::NedoCapNode',
            name='left_cap',
            namespace=topic_ns + '/left',
            parameters=[
                {'device': '/dev/left_camera'},
                {'frame_id': 'left_camera_frame'},
                {'pipeline': 'v4l2src device=/dev/left_camera ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! appsink'}
                ]
            ),
            # 右カメラ映像取得
            ComposableNode(
            package='nedo_capture',
            plugin='nedo_capture::NedoCapNode',
            name='right_cap',
            namespace=topic_ns + '/right',
            parameters=[
                {'device': '/dev/right_camera'},
                {'frame_id': 'right_camera_frame'},
                {'pipeline': 'v4l2src device=/dev/right_camera ! image/jpeg,width=3840,height=2160,framerate=30/1 ! jpegdec ! videoconvert ! appsink'}
                ]
            ),
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CompressNode',
                name='cap_comp',
                namespace=topic_ns
            ),
            # フロントカメラ情報
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CaminfoNode',
                name='front_caminfo',
                namespace=topic_ns + '/front',
                parameters=[{'calib_file': front_calib_file}]
            ),
            # 左カメラ情報
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CaminfoNode',
                name='left_caminfo',
                namespace=topic_ns + '/left',
                parameters=[{'calib_file': left_calib_file}]
            ),
            # 右カメラ情報
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CaminfoNode',
                name='right_caminfo',
                namespace=topic_ns + '/right',
                parameters=[{'calib_file': right_calib_file}]
            )
            #ComposableNode(
            #    package='nedo_capture',
            #    plugin='nedo_capture::CaminfoNode',
            #    namespace=topic_ns,
            #    parameters=[
            #        {'calib_file':calib_file}
            #    ]
            #)
        ]
    )  

    #ld.add_action(arg_calib_file)
    ld.add_action(arg_front_calib_file)
    ld.add_action(arg_left_calib_file)
    ld.add_action(arg_right_calib_file)
    ld.add_action(declare_lidar_type_cmd)
    #ld.add_action(hesai_launch)
    ld.add_action(livox_launch)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    
    return ld