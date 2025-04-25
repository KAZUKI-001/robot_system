import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,GroupAction,LogInfo
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.actions import Node,LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()

    default_calib_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'ost.yaml'
    )
    default_tf_json_path = os.path.join(
        get_package_share_directory('nedo_bringup'),
        'config',
        'calib.json'
    )

    livox_launch_path = get_package_share_directory('livox_ros2_avia')

    use_bag = LaunchConfiguration('use_bag')
    is_contour = LaunchConfiguration('is_contour')
    calib_file = LaunchConfiguration('calib_file')
    json_file = LaunchConfiguration('json_file_path')

    declare_bag_cmd = DeclareLaunchArgument('use_bag',
                                            default_value='true',
                                            description='use bag or not')

    declare_contour_cmd = DeclareLaunchArgument('is_contour',
                                                default_value='true')

    declare_calib_file_cmd = DeclareLaunchArgument('calib_file',
                                           default_value='file://'+default_calib_path,
                                           description='calibration yaml file')
    
    declare_tf_json_cmd = DeclareLaunchArgument('json_file_path',
                                                default_value=default_tf_json_path,
                                                description='cam2Lidar calib.json path')

    topic_ns = 'nedo'

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([livox_launch_path,'/launch/livox_lidar_launch.py']),
        condition = UnlessCondition(use_bag)
    )

    # グループ化された設定
    bag_group = GroupAction(
        actions=[
            #container for capture 
            Node(
                package='rclcpp_components',
                executable='component_container_mt',
                name='capture_container',
                output='screen',
            ),
            #container for localization
            Node(
                package='rclcpp_components',
                executable='component_container_mt',
                name='lidar_container',
                output='screen',
            ),
            #load capture components
            LoadComposableNodes(
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CompressNode',
                        name='cap_comp',
                        namespace=topic_ns
                    ),
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::CaminfoNode',
                        name='caminfo',
                        namespace=topic_ns,
                        parameters=[
                            {'calib_file':calib_file}
                        ]
                    ),
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DataSyncNode',
                        name='syncronizer',
                        namespace=topic_ns
                    )
                ]
            ),
            LoadComposableNodes(
                condition=UnlessCondition(is_contour),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::DetectorNode',
                        name='point_detector',
                        namespace=topic_ns
                    )
                ]
            ),
            LoadComposableNodes(
                condition=IfCondition(is_contour),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourNode',
                        name='contour_detector',
                        namespace=topic_ns
                    )
                ]
            ),
            #load localization components
            LoadComposableNodes(
                target_container='lidar_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ProjectionNode',
                        name='projection',
                        namespace=topic_ns
                    ),
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::LidarCamTF',
                        name='lidar_cam_tf',
                        namespace=topic_ns,
                        parameters=[
                            {'json_file_path':json_file}
                        ]
                    ),
                    ComposableNode(
                        package='nedo_localization',
                        plugin='nedo_localization::ContourMatcher',
                        name='contour_matcher',
                        namespace=topic_ns
                    )
                ]
            ),
            LoadComposableNodes(
                condition=IfCondition(use_bag),
                target_container='lidar_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_viz',
                        plugin='nedo_viz::ShowProjectionNode',
                        name='debug_img',
                        namespace=topic_ns
                    )
                ]
            ),
            Node(
                package='nedo_localization',
                executable = 'localizer_node',
                parameters=[os.path.join(get_package_share_directory('nedo_bringup'),'config','params.yaml')],
                output='screen'
            ),
            Node(
                condition = UnlessCondition(use_bag),
                package='nedo_navigation',
                executable='nedo_controller',
                output='screen'
            )
            ,
            LoadComposableNodes(
                condition=UnlessCondition(use_bag),
                target_container='capture_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nedo_capture',
                        plugin='nedo_capture::NedoCapNode',
                        name='capture_hardware',
                        namespace=topic_ns
                    )
                ]
            )
        ]
    )

    ld.add_action(declare_bag_cmd)
    ld.add_action(declare_contour_cmd)
    ld.add_action(declare_calib_file_cmd)
    ld.add_action(declare_tf_json_cmd)
    ld.add_action(LogInfo(msg=["is_contour: ", is_contour]))

    ld.add_action(livox_launch)
    ld.add_action(bag_group)
    
    return ld