from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control',
            executable='offboard_takeoff',
            name='offboard_takeoff_node',
            output='screen',
            parameters=[]   # 고도·속도 파라미터화는 추후 추가
        )
    ])
