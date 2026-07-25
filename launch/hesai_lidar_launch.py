from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    lidar_correction_file = os.path.join(
        get_package_share_directory("hesai_lidar"), 'config', 'PandarXT-16.csv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_type',
            default_value='both',
            description='Publish points, raw packets, or both',
        ),
        DeclareLaunchArgument(
            'pointcloud_reliability',
            default_value='reliable',
            description='QoS reliability for /points_raw',
        ),
        DeclareLaunchArgument(
            'pointcloud_qos_depth',
            default_value='1000',
            description='QoS history depth for /points_raw',
        ),
        DeclareLaunchArgument(
            'pointcloud_layout',
            default_value='compact',
            description='PointCloud2 layout: compact or legacy_pcl',
        ),
        Node(
            package='hesai_lidar',
            namespace='/',
            executable='hesai_lidar_node',
            name='hesai_node',
            output='screen',
            parameters=[
                {"pcap_file": ""},
                {"server_ip": "192.168.123.20"},
                {"lidar_recv_port": 2368},
                {"gps_port": 10110},
                {"start_angle": 0.0},
                {"lidar_type": "PandarXT-16"},
                {"frame_id": "hesai_lidar"},
                {"pcldata_type": 0},
                {
                    "publish_type": ParameterValue(
                        LaunchConfiguration('publish_type'), value_type=str
                    )
                },
                {
                    "pointcloud_reliability": ParameterValue(
                        LaunchConfiguration('pointcloud_reliability'),
                        value_type=str,
                    )
                },
                {
                    "pointcloud_qos_depth": ParameterValue(
                        LaunchConfiguration('pointcloud_qos_depth'),
                        value_type=int,
                    )
                },
                {
                    "pointcloud_layout": ParameterValue(
                        LaunchConfiguration('pointcloud_layout'),
                        value_type=str,
                    )
                },
                {"timestamp_type": "realtime"},
                {"data_type": ""},
                {"lidar_correction_file": lidar_correction_file},
                {"multicast_ip": ""},
                {"coordinate_correction_flag": False},
                {"fixed_frame": ""},
                {"target_frame": ""},
            ]
        )
    ])
