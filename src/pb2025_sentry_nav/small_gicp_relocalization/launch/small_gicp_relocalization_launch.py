# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    
    # 声明启动参数，允许从命令行传入路径和开关
    # 例如: ros2 launch ... enable_sc_relocalization:=True
    prior_pcd_file_arg = DeclareLaunchArgument('prior_pcd_file', default_value='', description='Global map PCD file')
    scd_directory_arg = DeclareLaunchArgument('scd_directory', default_value='', description='Directory for .scd files')
    pose_file_arg = DeclareLaunchArgument('pose_file', default_value='', description='History pose file')
    enable_sc_arg = DeclareLaunchArgument('enable_sc_relocalization', default_value='False', description='Enable ScanContext init')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    node = Node(
        package="small_gicp_relocalization",
        executable="small_gicp_relocalization_node",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=[
            {
                # === 核心算法参数 (YAML配置) ===
                "use_sim_time": True,
                "num_threads": 6,
                "num_neighbors": 15,
                "global_leaf_size": 2.0,
                "registered_leaf_size": 2.0,
                "max_dist_sq": 0.8,

                # === 坐标系设置 (YAML配置) ===
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "robot_base_frame": "gimbal_yaw",
                "lidar_frame": "front_mid360",

                # === ScanContext & NDT 参数 (YAML配置) ===
                "sc_dist_thresh": 0.5,
                "ndt_resolution": 1.0,
                "ndt_step_size": 0.1,
                "ndt_epsilon": 0.01,
                "ndt_max_iterations": 35,

                # === 动态参数 (来自 LaunchConfiguration) ===
                "enable_sc_relocalization": LaunchConfiguration("enable_sc_relocalization"),
                "prior_pcd_file": LaunchConfiguration("prior_pcd_file"),
                "scd_directory": LaunchConfiguration("scd_directory"),
                "pose_file": LaunchConfiguration("pose_file"),
            }
        ],
    )

    return LaunchDescription([
        prior_pcd_file_arg,
        scd_directory_arg,
        pose_file_arg,
        enable_sc_arg,
        node
    ])
