import os
import shutil
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def perform_cleanup(context, *args, **kwargs):
    """Wipes Gazebo cache to prevent 'ghost' transforms."""
    gz_cache = os.path.expanduser('~/.gz/sim/main/binary_cache/')
    ign_cache = os.path.expanduser('~/.ignition/sim/main/binary_cache/')
    for cache_path in [gz_cache, ign_cache]:
        if os.path.exists(cache_path):
            shutil.rmtree(cache_path)
            os.makedirs(cache_path)
    return [LogInfo(msg="[Cleanup] Gazebo cache purged.")]

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_controller')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')
    robot_file = os.path.join(pkg_share, 'models', 'grey_robot', 'model.sdf')
    models_path = os.path.join(pkg_share, 'models')

    # Environment variable for Gazebo models
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_path]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(robot_file).read(),
            # This adds "grey_robot/" to all links so they connect to the odom tree
            'frame_prefix': 'grey_robot/' 
        }]
    )

    return LaunchDescription([
        OpaqueFunction(function=perform_cleanup),
        set_gz_resource_path,

        # This forces the bridge and spawner to use Harmonic (GZ) instead of Fortress (IGN)
        SetEnvironmentVariable(name='GZ_VERSION', value='harmonic'),
        
        OpaqueFunction(function=perform_cleanup),
        set_gz_resource_path,

        # 1. Start Gazebo
        ExecuteProcess(cmd=['gz', 'sim', '-r', world_file], output='screen'),

        #2. Spawn Robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'grey_robot', '-file', robot_file, '-world', 'empty', '-x', '-6', '-y', '-11', '-z', '0.5'],
            output='screen'
        ),

        #spawn human model
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'human', '-file', os.path.join(pkg_share, 'models', 'human_model', 'model.sdf'), '-world', 'empty', '-x', '1.3', '-y', '-12.2', '-z', '0.67', '-Y', '-1.57'],
            output='screen'
        ),

        # 3. ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/empty/model/grey_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist', 
                '/model/grey_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/grey_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                
                # Suspension - Use @ for bidirectional or [ ] for specific
                '/model/grey_robot/joint/suspension_L/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/grey_robot/joint/suspension_R/cmd_force@std_msgs/msg/Float64@gz.msgs.Double',
                
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', 
                
            ],
            remappings=[
                ('/world/empty/model/grey_robot/joint_state', '/joint_states'),
                ('/model/grey_robot/odometry', '/odom'),
                ('/model/grey_robot/tf', '/tf'),
                ('/lidar', '/scan'),
            ],
            output='screen'
        ),

        # 8. Suspension Controller
        # Node(
        #     package='my_robot_controller',
        #     executable='active_suspension',
        #     output='screen'
        # ),

        robot_state_publisher_node,


    ])