from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = [{
        'frame_id': 'robot_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'publish_tf_odom': False,
        'subscribe_odom_info': False,
        'wait_imu_to_init': False,
        'use_sim_time': True
    }]


    remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera'),
        ('rgb/camera_info', '/camera_info'),
        ('depth/image', '/depth_camera'),
        ('odom', '/vehicle/odometry')
    ]


    return LaunchDescription([
        # Nodes to launch
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters = parameters,
            remappings = remappings
        ),


        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']
        ),


        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.15', '0.03', '0.002', '0.0', '0.0', '0.0', 'robot_link', 'x500_depth_0/OakD-Lite/base_link/IMX214']),
    ])
