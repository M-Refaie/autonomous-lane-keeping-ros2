import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro 

def generate_launch_description():
    pkg_name = 'diff_drive_pkg'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    robot_description_path_resolved = os.path.join(
        pkg_share_dir, 'urdf', 'diff_drive_robot.urdf.xacro'
    )
    world_file = os.path.join(pkg_share_dir, 'worlds', 'two_lane_track.sdf')

    def process_xacro(context):
        robot_description_content = xacro.process_file(robot_description_path_resolved).toxml()
        
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description_content, 
                             'use_sim_time': True}]
            )
        ]

    robot_state_publisher_node = OpaqueFunction(function=process_xacro)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments=[
            ('gz_args', ['-r ', world_file]) 
        ]
    )
    
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-name', 'diff_drive_robot',
                   '-topic', 'robot_description',
                   '-z', '0.0'], # Spawn low so gravity works
        output='screen'
    )
    
    # 6. Bridge ROS 2 <-> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # FIXED BRIDGE SYNTAX FOR SENSORS
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.JointState', 
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V', 
        ],
        output='screen'
    )
   
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge,
    ])