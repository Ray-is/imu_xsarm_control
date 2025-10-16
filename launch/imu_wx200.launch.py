from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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

    # Launch the bno055 driver, and tell ros to re-launch it if it is killed
    imu_config = os.path.join(
        get_package_share_directory('imu_arm_control'),
        'launch',
        'bno055_params.yaml'
        )
    
    bno055_driver_node=Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [imu_config],
        respawn=True,
        respawn_delay=2.0
    )

    # Launch this node
    imu_arm_control_node = Node(
        package='imu_arm_control',
        executable='imu_wx200_control'
    )


    
    return LaunchDescription([
        use_sim_arg,
        xsarm_control_launch_include,
        bno055_driver_node,
        imu_arm_control_node

    ])