from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    #urdf_path = os.path.join(get_package_share_path('tarobot_one'),
    #                         'description', 'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('tarobot_one'),
                             'rviz', 'teleop_sim.rviz')
    world = os.path.join(get_package_share_path('tarobot_one'),'worlds','outdoor_sim.world')
    
    #robot_description = ParameterValue(Command(['xacro  ', urdf_path]), value_type=str)

    #robot_state_publisher_node = Node(
    #    package="robot_state_publisher",
    #    executable="robot_state_publisher",
    #    parameters=[{'robot_description' : robot_description}]
    #)

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_path('tarobot_one'),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
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
        #robot_state_publisher_node,
        rsp,
        gazebo_world,
        spawn_entity,
        rviz2_node
    ])