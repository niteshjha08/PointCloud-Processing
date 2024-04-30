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
    # Launch CARLA
    CARLA_ROOT = os.environ['CARLA_ROOT']
    carla_ros_bridge_dir = get_package_share_directory('carla_ros_bridge')

    run_carla_cmd = ExecuteProcess(
        cmd=['sh',  CARLA_ROOT+'/'+'CarlaUE4.sh'],
        shell=True, output='screen')
    # Launch carla_ros_bridge on manual control mode
    launch_carla_ros_bridge_manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(carla_ros_bridge_dir, 'carla_ros_bridge_with_example_ego_vehicle.launch.py')
        ),
    )

    spawn_order_callback = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=run_carla_cmd,
            on_start=[TimerAction(
                period=12.0,
                actions=[launch_carla_ros_bridge_manual_control])],
        )
    )

    # launch rviz2
    rviz_node = Node(package='rviz2',
         namespace='',
         executable='rviz2',
         name='rviz2')
    ld = LaunchDescription()
    ld.add_action(run_carla_cmd)
    # ld.add_action(launch_carla_ros_bridge_manual_control)
    ld.add_action(spawn_order_callback)

    ld.add_action(rviz_node)

    return ld