from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topic_sub_launch_arg = DeclareLaunchArgument(
        "topic_sub", default_value=TextSubstitution(text="face_id"))
    topic_pub_launch_arg = DeclareLaunchArgument(
        "topic_pub", default_value=TextSubstitution(text="mask_eval"))
    eng_rebuild_launch_arg = DeclareLaunchArgument(
        "eng_rebuild", default_value=TextSubstitution(text="True"))
    eng_save_launch_arg = DeclareLaunchArgument(
        "eng_save", default_value=TextSubstitution(text="True"))
    eng_onnx_path_launch_arg = DeclareLaunchArgument(
        "eng_onnx_path", default_value=TextSubstitution(text="/opt/dev/DL_Models/face_rec/model_mask/resnet152.onnx"))
    eng_save_path_launch_arg = DeclareLaunchArgument(
        "eng_save_path", default_value=TextSubstitution(text="/opt/dev/DL_Models/face_rec/model_mask/mask_predict.rt"))
    eng_use_dla_launch_arg = DeclareLaunchArgument(
        "eng_use_dla", default_value=TextSubstitution(text="-1"))
    eng_fp_16_launch_arg = DeclareLaunchArgument(
        "eng_fp_16", default_value=TextSubstitution(text="True"))
    return LaunchDescription([
        topic_sub_launch_arg,
        topic_pub_launch_arg,
        eng_rebuild_launch_arg,
        eng_save_launch_arg  ,
        eng_onnx_path_launch_arg,
        eng_save_path_launch_arg,
        eng_use_dla_launch_arg,
        eng_fp_16_launch_arg,
        Node(
            package='rtface_pkg',
            namespace='',
            executable='mask_node',
            name='mask_Node',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"topic_sub": LaunchConfiguration('topic_sub'),
                 "topic_pub": LaunchConfiguration('topic_pub'),
                 "eng_rebuild": LaunchConfiguration('eng_rebuild'),
                 "eng_save": LaunchConfiguration('eng_save'),
                 "eng_onnx_path": LaunchConfiguration('eng_onnx_path'),
                 "eng_save_path": LaunchConfiguration('eng_save_path'),
                 "eng_use_dla": LaunchConfiguration('eng_use_dla'),
                 "eng_fp_16": LaunchConfiguration('eng_fp_16')
                 }
            ]
        )
    ])