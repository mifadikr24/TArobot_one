from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    package_name='tarobot_one'

    rviz_config_path = os.path.join(get_package_share_path(package_name),
                             'rviz', 'nav2_sim.rviz')
    world = os.path.join(get_package_share_path('tarobot_one'),'worlds','indoor_sim.world')
    
    gazebo_params_file = os.path.join(get_package_share_path(package_name),'config','gazebo_params.yaml')
    
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_path(package_name),'launch','rsp.launch.py')]),
                 launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
            )

    online_async = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_path(package_name),'launch','online_async_launch.py')])
            )
    
    nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_path(package_name),'launch','navigation_launch.py')])
            )
    
    localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_path(package_name),'launch','localization_launch.py')])
            )
    
    gazebo_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments = {'world' : world}.items()
            )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                arguments=['-topic', 'robot_description',
                            '-entity', 'robot'],
                output='screen')

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )
                        
    return LaunchDescription([
        rsp,
        online_async,
        nav,
        localization,
        gazebo_world,
        spawn_entity,
        rviz2_node
    ])