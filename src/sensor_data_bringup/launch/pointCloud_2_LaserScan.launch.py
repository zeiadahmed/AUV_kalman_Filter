from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    reading_pointcloud = Node(

        package='pointcloud_to_laserscan',
        node_executable='pointcloud_to_laserscan_node',
        node_name='pointcloud_to_laserscan',
        
        remappings=[
            ('/cloud_in', '/swift/sonar/PointCloud2'),
        ]
        # ,

        # parameters=[ {"target_frame": 'swift/sonar_beam_link'}]
        # , parameters=[{'depth_module.global_time_enabled': True}]
    )

    return LaunchDescription([
        reading_pointcloud
    ])
