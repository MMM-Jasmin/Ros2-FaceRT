from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    video_arg = DeclareLaunchArgument(
        "video", default_value=TextSubstitution(text="../test_videos/video_cap_720.mp4"))
    frequency_arg = DeclareLaunchArgument(
        "frequency", default_value=TextSubstitution(text="30.0"))
    return LaunchDescription([
        video_arg,
        frequency_arg,
        Node(
            package='rtface_pkg',
            namespace='',
            executable='cam_node',
            name='cam',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"depth": 1,
                 "cam_id": 0,
                 "show_camera": False,
                 "width": 800,
                 "height": 600,
                #  "frequency": 30.0,
                "frequency": LaunchConfiguration('frequency'),
                "video": LaunchConfiguration('video'),
                #  "video": "./video_cap_720.mp4",
                 "video_test": True
                 }
            ]
        )
    ])