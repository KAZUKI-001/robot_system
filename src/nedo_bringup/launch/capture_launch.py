from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container_name = 'test_container'
    ld = LaunchDescription()
    container = Node(
        name= container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        )
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::NedoCapNode',
                name='my_cap',
                namespace='test'
            ),
            ComposableNode(
                package='nedo_capture',
                plugin='nedo_capture::CompressNode',
                name='cap_comp',
                namespace='test'
            ),
            ComposableNode(
                package='nedo_localization',
                plugin='nedo_localization::DetectorNode',
                name='detector',
                namespace='test'
            )
        ]
    )
    ld.add_action(container)
    ld.add_action(load_composable_nodes)

    return ld