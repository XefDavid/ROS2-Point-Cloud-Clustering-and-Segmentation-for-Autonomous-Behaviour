import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('point_cloud_processing'),'rviz','data_viz.rviz')

    return LaunchDescription([
        Node(
            package='ros2_kitti_publishers', executable='kitti_publishers',output='screen',         
        ),
        # Node(
        #     package='point_cloud_processing', executable='segment',output='screen',
        #     arguments=['-d',config_file_path]
        # ),
    ])

   

    

    