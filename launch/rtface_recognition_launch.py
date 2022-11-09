from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    topic_sub_launch_arg = DeclareLaunchArgument(
        "topic_sub", default_value=TextSubstitution(text="/face_id"))
    topic_pub_launch_arg = DeclareLaunchArgument(
        "topic_pub", default_value=TextSubstitution(text="/rec_id"))
    topic_newPerson_sub_launch_arg = DeclareLaunchArgument(
        "topic_newPerson_sub", default_value=TextSubstitution(text="/new_person"))
    similarity_th_launch_arg = DeclareLaunchArgument(
        "similarity_th", default_value=TextSubstitution(text="0.3"))
    eng_embeddings_path_launch_arg = DeclareLaunchArgument(
        "eng_embeddings_path", default_value=TextSubstitution(text="/opt/dev/embeddings1"))
    eng_rebuild_launch_arg = DeclareLaunchArgument(
        "eng_rebuild", default_value=TextSubstitution(text="False"))
    eng_save_launch_arg = DeclareLaunchArgument(
        "eng_save", default_value=TextSubstitution(text="True"))
    eng_feature_onnx_path_launch_arg = DeclareLaunchArgument(
        "eng_feature_onnx_path", default_value=TextSubstitution(text="/opt/dev/DL_Models/face_rec/model_recognition/ms1mv2_r50.onnx"))
    eng_feature_save_path_launch_arg = DeclareLaunchArgument(
        "eng_feature_save_path", default_value=TextSubstitution(text="/opt/dev/engines/ffeature_32.rt"))
    eng_use_dla_launch_arg = DeclareLaunchArgument(
        "eng_use_dla", default_value=TextSubstitution(text="-1"))
    eng_fp_16_launch_arg = DeclareLaunchArgument(
        "eng_fp_16", default_value=TextSubstitution(text="True"))
    eng_attention_launch_arg = DeclareLaunchArgument(
        "eng_attention", default_value=TextSubstitution(text="False"))
    return LaunchDescription([
        topic_sub_launch_arg,
        topic_pub_launch_arg,
        topic_newPerson_sub_launch_arg,
        eng_embeddings_path_launch_arg,
        eng_rebuild_launch_arg,
        eng_save_launch_arg,
        eng_feature_onnx_path_launch_arg,
        eng_feature_save_path_launch_arg,
        eng_use_dla_launch_arg,
        eng_fp_16_launch_arg,
        eng_attention_launch_arg,
        similarity_th_launch_arg,
        Node(
            package='rtface_pkg',
            namespace='',
            executable='recognize_node',
            name='recognition_node',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"topic_sub": LaunchConfiguration('topic_sub'),
                 "topic_pub": LaunchConfiguration('topic_pub'),
                 "topic_newPerson_sub": LaunchConfiguration('topic_newPerson_sub'),
                 "similarity_th": LaunchConfiguration('similarity_th'),
                 "eng_embeddings_path": LaunchConfiguration('eng_embeddings_path'),
                 "eng_rebuild": LaunchConfiguration('eng_rebuild'),
                 "eng_save": LaunchConfiguration('eng_save'),
                 "eng_feature_onnx_path": LaunchConfiguration('eng_feature_onnx_path'),
                 "eng_feature_save_path": LaunchConfiguration('eng_feature_save_path'),
                 "eng_use_dla": LaunchConfiguration('eng_use_dla'),
                 "eng_fp_16": LaunchConfiguration('eng_fp_16'),
                 "eng_attention": LaunchConfiguration('eng_attention'),
                 }
            ]
        )
    ])