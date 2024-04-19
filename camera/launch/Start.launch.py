import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package's share directory
    package_share_directory = get_package_share_directory('camera')

    # Define the path to the parameter file
    param_file = os.path.join(package_share_directory, 'param', 'filter.yaml')

    # Declare a launch argument to specify the parameter file
    param_config_arg = DeclareLaunchArgument(
        'param',
        default_value=param_file,
        description='Path to the parameter file'
    )

    # Define the node to be launched
    WebcamPub_node = Node(
        package='camera',
        executable='WebcamPub',
        name='WebcamPub',
        parameters=[LaunchConfiguration('param')],
        output='screen'
    )
    
    ImageServiceServer_node = Node(
        package='camera',
        executable='ImageServiceServer',
        name='ImageServiceServer',
        parameters=[LaunchConfiguration('param')],
        output='screen'
    )

    # Create the launch description and specify the launch sequence
    return LaunchDescription([
        param_config_arg,
        WebcamPub_node,
        ImageServiceServer_node

    ])
