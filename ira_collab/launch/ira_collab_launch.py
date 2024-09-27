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
        DeclareLaunchArgument(
            'cam_port',
            default_value='4',
            description='Port number of the camera'
        ),
        DeclareLaunchArgument(
            'eyes_port',
            default_value='/dev/ttyACM0',
            description='Port number of the Arduino for the eyes'
        ),
        Node(
            package='ira_collab',
            executable='camera_node',
            name='camera_node',
            parameters=[
                {'sim': LaunchConfiguration('sim')}, 
                {'cam_port': '4'},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='ira_collab',
            executable='interaction_node',
            name='interaction_node',
            parameters=[
                {'sim': LaunchConfiguration('sim')},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='ira_collab',
            executable='arm_node',
            name='arm_node',
            parameters=[
                {'sim': LaunchConfiguration('sim')},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='ira_collab',
            executable='eye_node',
            name='eye_node',
            parameters=[
                {'sim': LaunchConfiguration('sim')},
                {'eyes_port': '/dev/ttyACM0'},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='ira_collab',
            executable='gpt_node',
            name='gpt_node',
            parameters=[
                {'sim': LaunchConfiguration('sim')},
                {'log_level': 'INFO'}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])
