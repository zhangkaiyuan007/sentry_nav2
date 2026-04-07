import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package="bringup").find("bringup")
    ################### user configure parameters for ros2 start ###################
    xfer_format = 1  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
    data_src = 0  # 0-lidar, others-Invalid data src
    publish_freq = 10.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.发布频率配置
    output_type = 0
    frame_id = "base_link"
    lvx_file_path = "/home/livox/livox_test.lvx"
    cmdline_bd_code = "livox0000000001"
    user_config_path = os.path.join(pkg_share, "config", "MID360_config.json")
    ################### user configure parameters for ros2 end #####################

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
    ]

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="log",
        parameters=livox_ros2_params,
    )

    super_lio_node = Node(
        package="super_lio",
        executable="super_lio_node",
        parameters=[
            os.path.join(pkg_share, "config", "mid360.yaml"),
        ],
        output="log",
    )

    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        parameters=[
            {
                "resolution": 0.05,
                "frame_id": "odom",
                "sensor_model/max_range": 15.0,
                "occupancy_min_z": 0.1,
                "occupancy_max_z": 0.8,
                "filter_ground": True,
            }
        ],
        remappings=[("/cloud_in", "/lio/cloud_world")],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "map.rviz")],
    )

    ld = LaunchDescription()
    ld.add_action(livox_driver)
    ld.add_action(super_lio_node)
    ld.add_action(octomap_node)
    ld.add_action(rviz2_node)

    return ld
