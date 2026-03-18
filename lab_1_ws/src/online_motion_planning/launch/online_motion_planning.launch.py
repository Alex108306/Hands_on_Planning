import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    node_1 = Node(
        package='online_motion_planning',
        executable='localization_node',
        name='localization_node',
        output='screen',
    )

    node_2 = Node(
        package='online_motion_planning',
        executable='grid_mapping',
        name='grid_mapping',
        output='screen',
    )

    node_3 = Node(
        package='online_motion_planning',
        executable='online_motion_planning_node',
        name='online_motion_planning_node',
        output='screen',
    )

    node_4 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('online_motion_planning'), 'rviz', 'online_motion_planning.rviz'])]
    )

    ld = LaunchDescription([
        # node_1,
        node_2,
        node_3,
    ])
    ld.add_action(node_4)
    
    return ld
