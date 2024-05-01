import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    config_filter_node = os.path.join(
        get_package_share_directory('point_cloud_proc'),
        'config',
        'preprocess_cloud_config.yaml'
        )
    # launch rviz2
    filter_cloud_node = Node(package='point_cloud_proc',
         namespace='',
         executable='preprocess_cloud_node',
         name='preprocess_cloud_node',
         parameters=[config_filter_node])
    ld = LaunchDescription()

    ld.add_action(filter_cloud_node)


    return ld