import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('open_manipulator_x_description'),
                    'launch',
                    'open_manipulator_x.launch.py'
                )
            )
        ),
        
        Node(
            package='robot_motion_service',
            executable='motion_service.py',
            name='motion_service',
            output='screen'
        ),
        
        Node(
            package='robot_motion_service',
            executable='Robot_Ui.py',
            name='robot_ui',
            output='screen'
        ),
        
        Node(
            package='robot_motion_service',
            executable='kinematics_service.py',
            name='kinematics_service',
            output='screen'
        )
    ])
