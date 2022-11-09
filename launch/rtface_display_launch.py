from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    topic_launch_arg = DeclareLaunchArgument(
        "topic", default_value=TextSubstitution(text="detections"))
    topic_addPerson_launch_arg = DeclareLaunchArgument(
        "topic_addPerson", default_value=TextSubstitution(text="add_person"))
    save_video_arg = DeclareLaunchArgument(
        "save_video", default_value=TextSubstitution(text="False"))
    width_arg = DeclareLaunchArgument(
        "width", default_value=TextSubstitution(text="640"))
    height_arg = DeclareLaunchArgument(
        "height", default_value=TextSubstitution(text="480"))
    frames_arg = DeclareLaunchArgument(
        "frames", default_value=TextSubstitution(text="300"))
    return LaunchDescription([
        topic_launch_arg,
        topic_addPerson_launch_arg,
        save_video_arg,
        width_arg,
        height_arg,
        frames_arg,
        Node(
            package='rtface_pkg',
            namespace='',
            executable='display_node',
            name='display_Node',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"topic": LaunchConfiguration('topic'),
                 "topic_addPerson": LaunchConfiguration('topic_addPerson'),
                 "save_video": LaunchConfiguration('save_video'),
                 "width": LaunchConfiguration('width'),
                 "height": LaunchConfiguration('height'),
                 "frames": LaunchConfiguration('frames')
                 }
            ]
        )
    ])