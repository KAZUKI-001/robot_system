from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, LoadComposableNodes
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    print("start camera calibration ...")
    print("optional: set device file name like follows... device_file:=/dev/videoXX")
    device_file = LaunchConfiguration('device_file')
    declare_device_file_cmd = DeclareLaunchArgument('device_file',default_value='/dev/video0')
    
    container_name = 'test_container'
    ld = LaunchDescription()
    container = Node(
        name= container_name,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        )
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::NedoCapNode',
                name='calib_cap',
                namespace='calibration',
                parameters=[
                    {'device':device_file}
                ]
            ),
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CompressNode',
                name='cap_comp',
                namespace='calibration'
            )
        ]
    )
    calib_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        output='screen',
        arguments=[
            '--size','7x9',
            '--square', '0.02'
        ],
        parameters=[
            {'camera':'/nedo_camera'}
        ],
        remappings=[
            ('image','/calibration/capture'),
        ]
    )
    ld.add_action(declare_device_file_cmd)
    ld.add_action(container)
    ld.add_action(load_composable_nodes)
    ld.add_action(calib_node)

    return ld


