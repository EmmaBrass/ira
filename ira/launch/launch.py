from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim', # Sim mode removes the need for arm connection.  Still assumes eyes, camera, and gpt connection.
            default_value='False',
            description='Run in simulation mode'
        ),
        Node(
            package='ira',
            executable='camera_node',
            name='camera_node',
            parameters=[{'sim': LaunchConfiguration('sim')}]
        ),
        Node(
            package='ira',
            executable='interaction_node',
            name='interaction_node',
            parameters=[{'sim': LaunchConfiguration('sim')}]
        ),
        Node(
            package='ira',
            executable='arm_node',
            name='arm_node',
            parameters=[{'sim': LaunchConfiguration('sim')}]
        ),
        Node(
            package='ira',
            executable='eye_node',
            name='eye_node',
            parameters=[{'sim': LaunchConfiguration('sim')}]
        ),
        Node(
            package='ira',
            executable='gpt_node',
            name='gpt_node',
            parameters=[{'sim': LaunchConfiguration('sim')}]
        )
    ])