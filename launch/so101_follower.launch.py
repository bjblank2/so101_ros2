from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for SO101 follower arm'
        ),
        DeclareLaunchArgument(
            'arm_id',
            default_value='follower_arm',
            description='Arm ID for namespace'
        ),
        DeclareLaunchArgument(
            'leader_arm_id',
            default_value='leader_arm',
            description='leader arm ID to follow'
        ),
        DeclareLaunchArgument(
            'max_relative_target',
            default_value='20.0',
            description='Maximum relative target movement (degrees)'
        ),
        DeclareLaunchArgument(
            'calibration_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('kiwi_robot'),
                'config',
                'so101_follower_calibration.yaml',
            ]),
            description='ROS 2 parameter file that defines follower arm calibration data'
        ),
        
        Node(
            package='so101_ros2',
            executable='so101_ros2_node',
            name='so101_follower',
            parameters=[{
                'mode': 'follower',
                'port': '/dev/ttyACM1',
                'arm_id': 'follower_arm',
                'peer_arm_id': 'leader_arm',
                'calibration_file': '/path/to/so101_follower_calibration.yaml',
            }],
            output='screen'
        ),
    ])

