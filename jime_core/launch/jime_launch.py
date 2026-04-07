import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # The folder where your index.html and host_robot_logic.py live
    core_path = '/home/ubuntu/ros2_ws/src/jime_core/jime_core'

    return LaunchDescription([
        # 1. Rosbridge (Android to Pi Communication)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),

        # 2. Web Video Server (Camera Stream)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen'
        ),

        # 3. HTTP Server (Port 8000)
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=core_path, 
            output='screen'
        ),

        # 4. Robot Logic (The Brain)
        ExecuteProcess(
            cmd=['python3', 'host_robot_logic.py'],
            cwd=core_path,
            output='screen'
        ),

        # 5. micro-ROS Bridge (Your Docker Wrapper Node)
        # This node runs 'docker run ...' so we don't need the native agent package
        Node(
            package='jime_core',
            executable='start_bridge',
            name='microros_bridge',
            output='screen'
        ),
            
    ])
