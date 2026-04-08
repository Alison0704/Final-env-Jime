import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # The folder where your index.html and host_robot_logic.py live
    core_path = '/home/ubuntu/ros2_ws/src/jime_core/jime_core'

    return LaunchDescription([
        # Rosbridge (Android to Pi Communication)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),
        # Allows HTML to talk to ROS2 via WebSockets
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge',
            parameters=[{'port': 9090}],
            output='screen'
        ),

        # Web Video Server (Camera Stream)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{'port': 8080}]
        ),
        # The Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'output_encoding': 'rgb8',
            }]
        ),
        # HTTP Server (Port 8000)
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=core_path, 
            output='screen'
        ),

        # Robot Logic (The Brain)
        ExecuteProcess(
            cmd=['python3', 'host_robot_logic.py'],
            cwd=core_path,
            output='screen'
        ),
        
       # # Micro-ROS Bridge (Your Docker Wrapper Node)
       # Node(
       #     package='jime_core',
       #     executable='start_bridge',
       #     name='microros_bridge',
       #     output='screen'
       #),
        # Ultrasonic Driver Node
        Node(
            package='jime_core',
            executable='ultrasonic',
            name='ultrasonic_node',
            output='screen'
        ),

        Node(
            package='jime_core',
            executable='bridge_node',
            name='esp32_bridge_driver',
            parameters=[{'port': '/dev/ttyUSB0'}]
        )
            
    ])
