from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

from cv_ros_nodes.workspace_paths import default_engine_path, default_number_engine_path


def _create_camera_nodes(context):
    res = LaunchConfiguration('resolution').perform(context)
    devs = LaunchConfiguration('camera_devices').perform(context)
    w, h = map(int, res.split(','))
    dev_list = [d.strip() for d in devs.split(',') if d.strip()]  # Remove empty strings
    
    # Check which camera devices actually exist
    existing_devices = []
    for i, dev in enumerate(dev_list):
        if os.path.exists(dev):
            existing_devices.append((i, dev))
            print(f"Camera {i} found: {dev}")
        else:
            print(f"Camera {i} NOT found (skipping): {dev}")
    
    if not existing_devices:
        print("WARNING: No camera devices found! CV pipeline will run without cameras.")
        return []
    
    print(f"Starting CV pipeline with {len(existing_devices)} camera(s)")
    
    # Create camera nodes only for existing devices with sequential startup delays
    nodes = []
    for cam_idx, (orig_idx, device_path) in enumerate(existing_devices):
        delay = cam_idx * 2.0  # 0s, 2s, 4s delays
        
        if delay == 0:
            # First camera: start immediately
            nodes.append(
                Node(
                    package='v4l2_camera',
                    executable='v4l2_camera_node',
                    name=f'camera{orig_idx}_node',
                    namespace=f'camera{orig_idx}',
                    parameters=[{
                        'video_device': device_path, 
                        'image_size': [w, h],
                        'output_encoding': 'yuv422_yuy2',  # Keep native YUYV, don't convert to RGB
                        'pixel_format': 'YUYV'             # Explicitly request YUYV from hardware
                    }]
                )
            )
        else:
            # Subsequent cameras: use timer delay to avoid USB race conditions
            nodes.append(
                TimerAction(
                    period=delay,
                    actions=[
                        LogInfo(msg=f'Starting Camera {orig_idx} ({delay}s delay to avoid race condition)'),
                        Node(
                            package='v4l2_camera',
                            executable='v4l2_camera_node',
                            name=f'camera{orig_idx}_node',
                            namespace=f'camera{orig_idx}',
                            parameters=[{
                                'video_device': device_path, 
                                'image_size': [w, h],
                                'output_encoding': 'yuv422_yuy2',  # Keep native YUYV, don't convert to RGB
                                'pixel_format': 'YUYV'             # Explicitly request YUYV from hardware
                            }]
                        )
                    ]
                )
            )
    
    return nodes


def generate_launch_description():
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='960,600',
        description='Camera resolution as W,H. Default 960x600 for native YUYV (no RGB conversion). Override with resolution:=480,360 for lower bandwidth if needed.'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to true when using simulation (Gazebo bridges publish /cameraN/image_raw). Skips v4l2 camera nodes.'
    )

    camera_devices_arg = DeclareLaunchArgument(
        'camera_devices',
        default_value='/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.4.3:1.0-video-index0',
        description='Comma-separated camera device paths. Can be 1, 2, or 3 cameras. Only existing devices will be started. Match set_camera_fps.sh; run monitor_camera_move.sh if paths change.'
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
        description='Staleness threshold for combiner (seconds). Raised to 20 for debugging; lower to 8 when pipeline is stable.'
    )

    enable_task4_arg = DeclareLaunchArgument(
        'enable_task4',
        default_value='false',
        description='Enable Task 4 supply drop processor'
    )

    enable_indicator_buoy_arg = DeclareLaunchArgument(
        'enable_indicator_buoy',
        default_value='false',
        description='Enable indicator buoy processor (red/green colour indicator buoy)'
    )

    enable_number_detection_arg = DeclareLaunchArgument(
        'enable_number_detection',
        default_value='false',
        description='Enable docking number detection (digits 1, 2, 3)'
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
        description='Run inference every N frames for front camera (camera1). 4 = ~1.25 inf/sec at 5 FPS; reduced to prevent CPU overload.'
    )
    inference_interval_sides_arg = DeclareLaunchArgument(
        'inference_interval_sides',
        default_value='6',
        description='Run inference every N frames for side cameras (camera0, camera2). 6 = ~0.83 inf/sec each at 5 FPS; reduced to prevent CPU overload.'
    )

    preprocess_fps_arg = DeclareLaunchArgument(
        'preprocess_fps',
        default_value='5',
        description='Max FPS for preprocessing output. Throttles 15 FPS camera input down to 5 FPS to reduce CPU load. Sequential camera startup prevents race condition. 0 = no limit.'
    )

    # Preprocessing nodes (throttle to preprocess_fps; driver keeps its own FPS)
    preprocess0 = Node(
        package='cv_ros_nodes',
        executable='vision_preprocessing',
        name='preprocess_camera0',
        arguments=['--camera_id', '0', '--max_fps', LaunchConfiguration('preprocess_fps')]
    )
    preprocess1 = Node(
        package='cv_ros_nodes',
        executable='vision_preprocessing',
        name='preprocess_camera1',
        arguments=['--camera_id', '1', '--max_fps', LaunchConfiguration('preprocess_fps')]
    )
    preprocess2 = Node(
        package='cv_ros_nodes',
        executable='vision_preprocessing',
        name='preprocess_camera2',
        arguments=['--camera_id', '2', '--max_fps', LaunchConfiguration('preprocess_fps')]
    )

    # Inference nodes (all start simultaneously - GPU serializes them automatically)
    inference0 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera0',
        arguments=[
            '--camera_id', '0',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold'),
            '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
            '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
            '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
            '--sim_image_rgb', 'false',
            '--inference_interval', LaunchConfiguration('inference_interval_sides'),
        ]
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
    inference2 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera2',
        arguments=[
            '--camera_id', '2',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold'),
            '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
            '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
            '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
            '--sim_image_rgb', 'false',
            '--inference_interval', LaunchConfiguration('inference_interval_sides'),
        ]
    )

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

    distance_scale_factor_arg = DeclareLaunchArgument(
        'distance_scale_factor',
        default_value='1.0',
        description='One-point calibration: scale factor = measured_distance_m / distance_specs (1.0 = no correction)'
    )
    
    task_arg = DeclareLaunchArgument(
        'task',
        default_value='3',
        description='Task number (2 or 3) - determines buoy reference dimensions. Task 2: all small buoys 0.5ft. Task 3: green/red/yellow 1ft, black 0.5ft.'
    )

    camera_ids_arg = DeclareLaunchArgument(
        'camera_ids',
        default_value='0,1,2',
        description='Comma-separated camera IDs for combiner/indicator/task4 (e.g. 1 for single-camera).'
    )
    
    # Final CV output: /combined/detection_info_with_distance (bearing, elevation, distance per detection)
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
        use_sim_arg,
        camera_devices_arg,
        engine_path_arg,
        conf_threshold_arg,
        staleness_threshold_arg,
        enable_task4_arg,
        enable_indicator_buoy_arg,
        enable_number_detection_arg,
        number_detection_engine_arg,
        number_conf_threshold_arg,
        inference_interval_front_arg,
        inference_interval_sides_arg,
        preprocess_fps_arg,
        distance_scale_factor_arg,
        task_arg,
        camera_ids_arg,
        LogInfo(msg='Starting computer vision pipeline...'),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('use_sim')),
            actions=[OpaqueFunction(function=_create_camera_nodes)]
        ),
        preprocess0,
        preprocess1,
        preprocess2,
        inference0,
        inference1,
        inference2,
        combiner,
        task4_supply_processor,
        indicator_buoy_processor,
        maritime_distance_estimator,
        LogInfo(msg='All computer vision nodes started!')
    ])
