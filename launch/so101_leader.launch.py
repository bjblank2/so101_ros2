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
            description='Serial port for SO101 leader arm'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Joint state publish rate (Hz)'
        ),
        DeclareLaunchArgument(
            'arm_id',
            default_value='leader_arm',
            description='Arm ID for namespace'
        ),
        DeclareLaunchArgument(
            'calibration_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('so101_ros2'),
                'params',
                'so101_leader_calibration.yaml',
            ]),
            description='ROS 2 parameter file that defines leader arm calibration data'
        ),
        
        # Leader node
        Node(
            package='so101_ros2',
            executable='so101_ros2_node',
            name='so101_leader',
            parameters=[
                LaunchConfiguration('calibration_params'),
                {
                    'mode': 'leader',
                    'port': LaunchConfiguration('port'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                    'arm_id': LaunchConfiguration('arm_id'),
                    'peer_arm_id': 'follower_arm',
                }
            ],
            output='screen'
        ),
    ])

