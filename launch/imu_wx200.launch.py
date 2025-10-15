from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    # Launch argument to use real or fake hardware
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        choices=('true', 'false'),
        default_value='false',
        description="Choice between using the real arm or a fake, simulated one."
    )
    use_sim = LaunchConfiguration('use_sim')


    # Launch the interbotix arm controller interface
    xsarm_control_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': "wx200",
            'use_rviz': 'true',
            'use_sim': use_sim,
        }.items(),
    )

    # Launch the bno055 driver
    bno055_driver_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bno055'),
                'launch',
                'bno055.launch.py'
            ])
        ]),
    )

    # Launch this node
    imu_arm_control_node = Node(
        package='imu_arm_control',
        executable='imu_wx200_control'
    )


    
    return LaunchDescription([
        use_sim_arg,
        xsarm_control_launch_include,
        bno055_driver_launch_include,
        imu_arm_control_node

    ])