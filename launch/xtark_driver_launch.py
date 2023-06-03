import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # """Launch file which brings up visual slam node configured for RealSense."""
    static_tf_baselink_lidar_link = Node(
        name='tf_static',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.12', '0', '0', '3.14', '0', '0', 'base_footprint', 'laser']
    )

    static_tf_baselink_basefootprint_link = Node(
        name='tf_static',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    xtark_driver = Node(
        name='xtark_driver',
        package='xtark_driver',
        executable='xtark_driver_node',
        output='screen',
        shell=True,
        parameters=[{
                    'port_name': "/dev/ttyTHS0",
                    'odom_frame': "odom",
                    'base_frame': "base_footprint",
                    'imu_frame': "base_imu_link",

                    'linear_correction_factor': 0.8,
                    'servo_bias': -4
                    
                    }]
    )


    return launch.LaunchDescription([xtark_driver,static_tf_baselink_lidar_link,static_tf_baselink_basefootprint_link])