import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    world_file_name = 'smaze2d.world'
    world_path = os.path.join(get_package_share_directory('mobile_robotics_hschoon'), 'worlds', world_file_name)
  
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': world_path}.items()
             )
             
    xacro_file = os.path.join(get_package_share_directory('mobile_robotics_hschoon'),
                              'urdf',
                              'mobile_robotics_hschoon.xacro')
                              
    rviz_file = os.path.join(get_package_share_directory('mobile_robotics_hschoon'),
                              'rviz',   
                              'mobile_robotics_hschoon.rviz')         
                                                
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    position = [0.6, 0.8, 1]                  
    orientation = [0.0, 0.0, -0.7]

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'simple_robot',
                                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
               			   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),],
                        output='screen',
                        parameters=[{'use_sim_time': True}])

    rviz_entity = Node(package='rviz2', executable='rviz2',
                        arguments=['-d', rviz_file],
                        parameters=[{'use_sim_time': True}])
                        
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    load_simple_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'simple_diff_drive_controller'],
        output='screen'
    )    
    
    node_test = Node(
       package='mobile_robotics_hschoon',
       executable='find_yellow.py',
       output='screen',
       name='find_yellow_node',       
    )
    
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_simple_diff_drive_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        rviz_entity,
        node_test
    ])
