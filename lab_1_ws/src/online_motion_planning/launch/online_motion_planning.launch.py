import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mode = LaunchConfiguration('mode')
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Run profile: sim or real',
    )
    
    node_1 = Node(
        package='online_motion_planning',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{'mode': mode}],
    )

    node_2 = Node(
        package='online_motion_planning',
        executable='grid_mapping',
        name='grid_mapping',
        output='screen',
        parameters=[{'mode': mode}],
    )

    node_3 = Node(
        package='online_motion_planning',
        executable='online_motion_planning_node',
        name='online_motion_planning_node',
        output='screen',
        parameters=[{'mode': mode}],
    )

    node_5 = Node(
        package='online_motion_planning',
        executable='frontier_exploration_node',
        name='frontier_exploration_node',
        output='screen',
        parameters=[{'mode': mode}],
    )

    node_4_sim = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sim',
        condition=LaunchConfigurationEquals('mode', 'sim'),
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('online_motion_planning'), 'rviz', 'rviz_HOP.rviz']
            ),
        ],
    )

    node_4_real = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_real',
        condition=LaunchConfigurationEquals('mode', 'real'),
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('online_motion_planning'), 'rviz', 'rviz_HOP_real.rviz']
            ),
        ],
    )

    ld = LaunchDescription([
        mode_arg,
        node_1,
        node_2,
        node_3,
        node_5,
    ])
    ld.add_action(node_4_sim)
    ld.add_action(node_4_real)
    
    return ld
