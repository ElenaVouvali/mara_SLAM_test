import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))
    
    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'))

    livox_to_pointcloud = launch_ros.actions.Node(
            #name='livox_to_pointcloud2',
            package='livox_to_pointcloud2',
      	    executable='livox_to_pointcloud2_node',
            remappings=[
            ('livox_pointcloud', '/mid70/front_scan'),
            ('converted_pointcloud2', '/mid70/front_scan/pcl')
            ]
	)
        
    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[('/input_cloud','/mid70/front_scan/pcl'),('/imu','/imu/data')],
        output='screen'
        )

    tf1 = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.925','0','0.948','0','0','0','1','mara_base_footprint','front_laser']
        )

    tf2 = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'mara_base_footprint']
        )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen'
        )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir]
        )


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        livox_to_pointcloud,
        mapping,
        tf1,
        tf2,
        graphbasedslam,
        rviz,
            ])
