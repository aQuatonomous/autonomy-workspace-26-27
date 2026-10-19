"""Launch CV pipeline for simulation: no v4l2 cameras; /cameraN/image_raw from Gazebo bridges.
Use single_camera:=true on Jetson when running sim + CV to avoid GPU OOM (NvMapMemAlloc error 12). Single-camera mode runs only camera1 (center)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from cv_ros_nodes.workspace_paths import default_engine_path, default_number_engine_path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='1920,1200', description='Camera resolution W,H'),
        DeclareLaunchArgument('engine_path', default_value=default_engine_path(), description='TensorRT engine'),
        DeclareLaunchArgument('conf_threshold', default_value='0.25', description='Confidence threshold'),
        DeclareLaunchArgument('staleness_threshold', default_value='1.0', description='Combiner staleness (s)'),
        DeclareLaunchArgument('enable_task4', default_value='false'),
        DeclareLaunchArgument('enable_indicator_buoy', default_value='false'),
        DeclareLaunchArgument('enable_number_detection', default_value='false'),
        DeclareLaunchArgument('number_detection_engine', default_value=default_number_engine_path()),
        DeclareLaunchArgument('number_conf_threshold', default_value='0.25'),
        DeclareLaunchArgument('distance_scale_factor', default_value='1.0'),
        DeclareLaunchArgument('task', default_value='3',
                              description='Task number (2 or 3) - determines buoy reference dimensions. Task 2: all small buoys 0.5ft. Task 3: green/red/yellow 1ft, black 0.5ft.'),
        DeclareLaunchArgument('single_camera', default_value='false',
                              description='If true, run only camera1/center (preprocess + inference). Use on Jetson to avoid GPU OOM when sim + CV run together.'),
        DeclareLaunchArgument('camera_ids', default_value='0,1,2',
                              description='Comma-separated camera IDs for combiner/indicator/task4. Pass camera_ids:=1 when single_camera:=true.'),
        DeclareLaunchArgument('inference_interval', default_value='1',
                              description='Run inference every N frames (1=every frame). Use 2 or 3 for smoother display FPS on Jetson.'),
        DeclareLaunchArgument('sim_image_rgb', default_value='true',
                              description='Treat images as RGB (Gazebo/ros_gz_bridge often publish RGB). Use false if sim images look wrong.'),
        DeclareLaunchArgument('draw_stale_boxes', default_value='true',
                              description='When inference_interval>1, draw last detections on every frame for smooth display (boxes may lag slightly when moving).'),
        DeclareLaunchArgument('image_encoding', default_value='rgb8', description='Preprocess encoding: rgb8 for sim, bgr8 for real cameras.'),
        LogInfo(msg='CV pipeline for simulation (no cameras; expect /camera0..2/image_raw from bridges).'),
        Node(package='cv_ros_nodes', executable='vision_preprocessing', name='preprocess_camera0',
             arguments=['--camera_id', '0', '--image_encoding', LaunchConfiguration('image_encoding')],
             condition=UnlessCondition(LaunchConfiguration('single_camera'))),
        Node(package='cv_ros_nodes', executable='vision_preprocessing', name='preprocess_camera1',
             arguments=['--camera_id', '1', '--image_encoding', LaunchConfiguration('image_encoding')]),
        Node(package='cv_ros_nodes', executable='vision_preprocessing', name='preprocess_camera2',
             arguments=['--camera_id', '2', '--image_encoding', LaunchConfiguration('image_encoding')],
             condition=UnlessCondition(LaunchConfiguration('single_camera'))),
        Node(
            package='cv_ros_nodes', executable='vision_inference', name='inference_camera0',
            arguments=['--camera_id', '0', '--engine_path', LaunchConfiguration('engine_path'),
                      '--conf_threshold', LaunchConfiguration('conf_threshold'),
                      '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
                      '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
                      '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
                      '--inference_interval', LaunchConfiguration('inference_interval'),
                      '--sim_image_rgb', LaunchConfiguration('sim_image_rgb'),
                      '--draw_stale_boxes', LaunchConfiguration('draw_stale_boxes')],
            condition=UnlessCondition(LaunchConfiguration('single_camera'))
        ),
        Node(
            package='cv_ros_nodes', executable='vision_inference', name='inference_camera1',
            arguments=['--camera_id', '1', '--engine_path', LaunchConfiguration('engine_path'),
                      '--conf_threshold', LaunchConfiguration('conf_threshold'),
                      '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
                      '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
                      '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
                      '--inference_interval', LaunchConfiguration('inference_interval'),
                      '--sim_image_rgb', LaunchConfiguration('sim_image_rgb'),
                      '--draw_stale_boxes', LaunchConfiguration('draw_stale_boxes')]
        ),
        Node(
            package='cv_ros_nodes', executable='vision_inference', name='inference_camera2',
            arguments=['--camera_id', '2', '--engine_path', LaunchConfiguration('engine_path'),
                      '--conf_threshold', LaunchConfiguration('conf_threshold'),
                      '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
                      '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
                      '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
                      '--inference_interval', LaunchConfiguration('inference_interval'),
                      '--sim_image_rgb', LaunchConfiguration('sim_image_rgb'),
                      '--draw_stale_boxes', LaunchConfiguration('draw_stale_boxes')],
            condition=UnlessCondition(LaunchConfiguration('single_camera'))
        ),
        Node(package='cv_ros_nodes', executable='vision_combiner', name='detection_combiner',
             arguments=['--staleness_threshold', LaunchConfiguration('staleness_threshold'),
                        '--camera_ids', LaunchConfiguration('camera_ids')]),
        Node(package='cv_ros_nodes', executable='task4_supply_processor', name='task4_supply_processor',
             arguments=['--camera_ids', LaunchConfiguration('camera_ids')],
             condition=IfCondition(LaunchConfiguration('enable_task4'))),
        Node(package='cv_ros_nodes', executable='indicator_buoy_processor', name='indicator_buoy_processor',
             arguments=['--camera_ids', LaunchConfiguration('camera_ids')],
             condition=IfCondition(LaunchConfiguration('enable_indicator_buoy'))),
        Node(package='cv_ros_nodes', executable='maritime_distance_estimator', name='maritime_distance_estimator',
             parameters=[{
                 'distance_scale_factor': LaunchConfiguration('distance_scale_factor'),
                 'task': LaunchConfiguration('task')
             }]),
        LogInfo(msg='CV sim launch complete.')
    ])
