from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    topic_sub_launch_arg = DeclareLaunchArgument(
        "topic_sub", default_value=TextSubstitution(text="face_id"))
    topic_sub_addPerson_launch_arg = DeclareLaunchArgument(
        "topic_sub_addPerson", default_value=TextSubstitution(text="add_person"))
    topic_pub_launch_arg = DeclareLaunchArgument(
        "topic_pub", default_value=TextSubstitution(text="new_person"))
    return LaunchDescription([
        topic_sub_launch_arg,
        topic_sub_addPerson_launch_arg,
        topic_pub_launch_arg,
        Node(
            package='rtface_pkg',
            namespace='',
            executable='addPerson_node',
            name='Add_person_to_rec',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"topic_sub": LaunchConfiguration('topic_sub'),
                 "topic_sub_addPerson": LaunchConfiguration('topic_sub_addPerson'),
                 "topic_pub": LaunchConfiguration('topic_pub')
                 }
            ]
        )
    ])