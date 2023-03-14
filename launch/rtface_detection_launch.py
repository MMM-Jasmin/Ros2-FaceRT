from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    cam_topic_launch_arg = DeclareLaunchArgument(
        "cam_topic", default_value=TextSubstitution(text="/background/color_limited"))
    mask_topic_launch_arg = DeclareLaunchArgument(
        "mask_topic", default_value=TextSubstitution(text="/mask_eval"))
    rec_topic_launch_arg = DeclareLaunchArgument(
        "rec_topic", default_value=TextSubstitution(text="/rec_id"))
    emotion_topic_launch_arg = DeclareLaunchArgument(
        "emotion_topic", default_value=TextSubstitution(text="/emotion_id"))
    faceId_topic_name_launch_arg = DeclareLaunchArgument(
        "faceId_topic_name", default_value=TextSubstitution(text="/face_id"))
    detection_topic_launch_arg = DeclareLaunchArgument(
        "detection_topic", default_value=TextSubstitution(text="/detections"))
    json_topic_launch_arg = DeclareLaunchArgument(
        "json_topic", default_value=TextSubstitution(text="/face_det/json_out"))
    fps_topic_launch_arg = DeclareLaunchArgument(
        "fps_topic", default_value=TextSubstitution(text="/face_det/fps"))
    max_fps_launch_arg = DeclareLaunchArgument(
        "max_fps", default_value=TextSubstitution(text="30.0"))    
    face_pub_size_launch_arg = DeclareLaunchArgument(
        "face_pub_size", default_value=TextSubstitution(text="224"))
    fp_duration_launch_arg = DeclareLaunchArgument(
        "fp_duration", default_value=TextSubstitution(text="500"))
    rotate_image_launch_arg = DeclareLaunchArgument(
        "rotate_image", default_value=TextSubstitution(text="False"))
    use_frameset_launch_arg = DeclareLaunchArgument(
        "use_frameset", default_value=TextSubstitution(text="False"))
    eng_weights_path_launch_arg = DeclareLaunchArgument(
        "eng_weights_path", default_value=TextSubstitution(text="/opt/dev/DL_Models/face_rec/model_retina/retinaface.wts"))
    eng_save_path_launch_arg = DeclareLaunchArgument(
        "eng_save_path", default_value=TextSubstitution(text="/opt/dev/engines/retina_detection_32.rt"))
    eng_use_dla_launch_arg = DeclareLaunchArgument(
        "eng_use_dla", default_value=TextSubstitution(text="-1"))
    eng_fp_16_launch_arg = DeclareLaunchArgument(
        "eng_fp_16", default_value=TextSubstitution(text="True"))

    return LaunchDescription([
        cam_topic_launch_arg,
        mask_topic_launch_arg,
        rec_topic_launch_arg,
        emotion_topic_launch_arg,
        faceId_topic_name_launch_arg,
        detection_topic_launch_arg,
        json_topic_launch_arg,
        fps_topic_launch_arg,
        max_fps_launch_arg,
        face_pub_size_launch_arg,
        fp_duration_launch_arg,
        rotate_image_launch_arg,
        use_frameset_launch_arg,
        eng_weights_path_launch_arg,
        eng_save_path_launch_arg,
        eng_use_dla_launch_arg,
        eng_fp_16_launch_arg,
        Node(
            package='rtface_pkg',
            namespace='',
            executable='detect_node',
            name='detection_node',
            output="screen",
            emulate_tty=True,
            parameters=[
                {"cam_topic": LaunchConfiguration('cam_topic'),
                 "mask_topic": LaunchConfiguration('mask_topic'),
                 "rec_topic":LaunchConfiguration('rec_topic'),
                 "emotion_topic":LaunchConfiguration('emotion_topic'),
                 "faceId_topic_name":LaunchConfiguration('faceId_topic_name'),
                 "detection_topic": LaunchConfiguration('detection_topic'),
                 "json_topic": LaunchConfiguration('json_topic'),
                 "fps_topic": LaunchConfiguration('fps_topic'),
                 "max_fps": LaunchConfiguration('max_fps'),
                 "face_pub_size": LaunchConfiguration('face_pub_size'),
                 "fp_duration": LaunchConfiguration('fp_duration'),
                 "rotate_image": LaunchConfiguration('rotate_image'),
                 "use_frameset": LaunchConfiguration('use_frameset'),
                 "eng_weights_path": LaunchConfiguration('eng_weights_path'),
                 "eng_save_path": LaunchConfiguration('eng_save_path'),
                 "eng_use_dla": LaunchConfiguration('eng_use_dla'),
                 "eng_fp_16": LaunchConfiguration('eng_fp_16')
                 }
            ]
        )
    ])
