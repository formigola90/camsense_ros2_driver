from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'camsense_ros2_driver'

    # Configure the robot state publisher node
    node_camsense_ros2_driver = Node(
        package='camsense_ros2_driver',
        executable='camsense_publisher',
        output='screen'
    )
    
    
    restart_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_camsense_ros2_driver,
            on_exit=[node_camsense_ros2_driver],
        )
    )

    return LaunchDescription([
    	node_camsense_ros2_driver,
    	restart_node
    ])
