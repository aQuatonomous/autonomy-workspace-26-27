from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

from cv_ros_nodes.workspace_paths import default_engine_path, default_number_engine_path


def _create_camera1_node(context):
    """Create only camera1 (middle) camera node"""
    res = LaunchConfiguration('resolution').perform(context)
    camera1_device = LaunchConfiguration('camera1_device').perform(context)
    w, h = map(int, res.split(','))
    
    if not os.path.exists(camera1_device):
        print(f"ERROR: Camera1 device NOT found: {camera1_device}")
        return []
    
    print(f"Single camera mode: Camera1 found: {camera1_device}")
    
    return [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera1_node',
            namespace='camera1',
            parameters=[{
                'video_device': camera1_device, 
                'image_size': [w, h],
                'output_encoding': 'yuv422_yuy2',  # Keep native YUYV, don't convert to RGB
                'pixel_format': 'YUYV'             # Explicitly request YUYV from hardware
            }]
        )
    ]


def generate_launch_description():
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='960,600',
        description='Camera resolution as W,H. Default 960x600 for native YUYV.'
    )

    camera1_device_arg = DeclareLaunchArgument(
        'camera1_device',
        default_value='/dev/v4l/by-path/platform-3610000.usb-usb-0:1.1:1.0-video-index0',
        description='Camera1 (middle) device path. Only this camera will run.'
    )

    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value=default_engine_path(),
        description='Path to TensorRT engine file'
    )

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.1',
        description='Confidence threshold for detections'
    )

    staleness_threshold_arg = DeclareLaunchArgument(
        'staleness_threshold',
        default_value='20.0',
        description='Staleness threshold for combiner (seconds).'
    )

    enable_task4_arg = DeclareLaunchArgument(
        'enable_task4',
        default_value='false',
        description='Enable Task 4 supply drop processor'
    )

    enable_indicator_buoy_arg = DeclareLaunchArgument(
        'enable_indicator_buoy',
        default_value='false',
        description='Enable indicator buoy processor'
    )

    enable_number_detection_arg = DeclareLaunchArgument(
        'enable_number_detection',
        default_value='false',
        description='Enable docking number detection'
    )
    number_detection_engine_arg = DeclareLaunchArgument(
        'number_detection_engine',
        default_value=default_number_engine_path(),
        description='Path to number detection TensorRT engine'
    )
    number_conf_threshold_arg = DeclareLaunchArgument(
        'number_conf_threshold',
        default_value='0.25',
        description='Confidence threshold for number detection'
    )
    
    inference_interval_front_arg = DeclareLaunchArgument(
        'inference_interval_front',
        default_value='4',
        description='Run inference every N frames for camera1.'
    )

    preprocess_fps_arg = DeclareLaunchArgument(
        'preprocess_fps',
        default_value='5',
        description='Max FPS for preprocessing output.'
    )

    distance_scale_factor_arg = DeclareLaunchArgument(
        'distance_scale_factor',
        default_value='1.0',
        description='Distance scale factor'
    )
    
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='3',
        description='Task number (2 or 3) - determines buoy reference dimensions. Task 2: all small buoys 0.5ft. Task 3: green/red/yellow 1ft, black 0.5ft.'
    )

    camera_ids_arg = DeclareLaunchArgument(
        'camera_ids',
        default_value='1',
        description='Comma-separated camera IDs for combiner/indicator/task4 (single-camera: 1 only).'
    )

    # Only camera1 preprocessing and inference nodes
    preprocess1 = Node(
        package='cv_ros_nodes',
        executable='vision_preprocessing',
        name='preprocess_camera1',
        arguments=['--camera_id', '1', '--max_fps', LaunchConfiguration('preprocess_fps')]
    )

    inference1 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera1',
        arguments=[
            '--camera_id', '1',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold'),
            '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
            '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
            '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
            '--sim_image_rgb', 'false',
            '--inference_interval', LaunchConfiguration('inference_interval_front'),
        ]
    )

    # Combiner (only subscribes/publishes for camera_ids)
    combiner = Node(
        package='cv_ros_nodes',
        executable='vision_combiner',
        name='detection_combiner',
        arguments=['--staleness_threshold', LaunchConfiguration('staleness_threshold'),
                   '--camera_ids', LaunchConfiguration('camera_ids')]
    )

    task4_supply_processor = Node(
        package='cv_ros_nodes',
        executable='task4_supply_processor',
        name='task4_supply_processor',
        arguments=['--camera_ids', LaunchConfiguration('camera_ids')],
        condition=IfCondition(LaunchConfiguration('enable_task4'))
    )

    indicator_buoy_processor = Node(
        package='cv_ros_nodes',
        executable='indicator_buoy_processor',
        name='indicator_buoy_processor',
        arguments=['--camera_ids', LaunchConfiguration('camera_ids')],
        condition=IfCondition(LaunchConfiguration('enable_indicator_buoy'))
    )

    maritime_distance_estimator = Node(
        package='cv_ros_nodes',
        executable='maritime_distance_estimator',
        name='maritime_distance_estimator',
        parameters=[{
            'distance_scale_factor': LaunchConfiguration('distance_scale_factor'),
            'task': LaunchConfiguration('task')
        }]
    )

    return LaunchDescription([
        resolution_arg,
        camera1_device_arg,
        engine_path_arg,
        conf_threshold_arg,
        staleness_threshold_arg,
        enable_task4_arg,
        enable_indicator_buoy_arg,
        enable_number_detection_arg,
        number_detection_engine_arg,
        number_conf_threshold_arg,
        inference_interval_front_arg,
        preprocess_fps_arg,
        distance_scale_factor_arg,
        task_arg,
        camera_ids_arg,
        LogInfo(msg='Starting single camera (camera1) CV pipeline...'),
        # Start camera node first
        OpaqueFunction(function=_create_camera1_node),
        # Wait 3s for camera to initialize before starting preprocessing
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='Starting preprocessing node...'),
                preprocess1,
            ]
        ),
        # Wait 2s more for preprocessing to start before inference
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='Starting inference node...'),
                inference1,
            ]
        ),
        # Wait 2s more before starting combiner and distance estimator
        TimerAction(
            period=7.0,
            actions=[
                LogInfo(msg='Starting combiner and distance estimator...'),
                combiner,
                maritime_distance_estimator,
                task4_supply_processor,
                indicator_buoy_processor,
            ]
        ),
        LogInfo(msg='Single camera CV pipeline launch sequence initiated!')
    ])