import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    
    		launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/publisher/camera/left/image_raw'],
            output='screen'
        ),
        Node(
            package='publisher',
            namespace='publisher',
            executable='image_publisher',
            name='publisher',
            output='screen',
            arguments=['/home/jaouad/dev_ws_rosbags/src/dataset_publisher_ros2/data/images/', 'image_0/', 'image_0/', '10'] #[main folder path, sub_folder_left, sub_folder_right, hz] (if mono put sub_folder_left=sub_folder_right)
        )        
    ])    
