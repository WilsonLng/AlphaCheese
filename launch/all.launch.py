from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'roboflow_api_key',
            default_value=EnvironmentVariable('ROBOFLOW_API_KEY')
        ),
        Node(
            package='motion_pkg',
            # executable='moveAndTake.py',
            executable='moveAndInverseKinematics.py',
            name='move_and_take'
        ),
        Node(
            package='comp_vision',
            executable='chessVision.py',
            name='chess_vision',
            environment={
                'ROBOFLOW_API_KEY': LaunchConfiguration('roboflow_api_key')
            }
        ),
        Node(
            package='algorithm',
            executable='chessAlgo.py',
            name='chess_algo'
        ),
        # Add any additional nodes or configurations here
    ])