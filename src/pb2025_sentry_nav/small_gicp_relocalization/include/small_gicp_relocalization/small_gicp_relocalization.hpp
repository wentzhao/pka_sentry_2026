#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "rm_utils/heartbeat.hpp"

// 只引入 Eigen Core，减少编译时间
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace pka;

namespace small_gicp_relocalization
{

// 前置声明：隐藏 ScanContext 实现细节
class SCManager;

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);
  ~SmallGicpRelocalizationNode();

private:
  // 回调
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  // 核心功能
  void loadGlobalMap(const std::string & file_name);
  void loadScanContextData();
  void performRegistration(); // GICP 追踪
  bool performGlobalLocalization(); // SC 全局初始化
  void refineInitialization(const Eigen::Isometry3d& initial_guess_map_to_base); // NDT 粗配准
  void publishTransform();

  // ROS 句柄
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  HeartBeatPublisher::SharedPtr heartbeat_;

  // --- 参数变量 ---
  // GICP & 系统
  bool use_sim_time_;
  int num_threads_;
  int num_neighbors_;
  double global_leaf_size_;
  double registered_leaf_size_;
  double max_dist_sq_;
  
  // TF 坐标系
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  
  // 文件路径
  std::string prior_pcd_file_;
  std::string scd_directory_;
  std::string pose_file_;

  // SC & NDT 调优参数
  bool enable_sc_relocalization_;
  double sc_dist_thresh_;
  double ndt_resolution_;
  double ndt_step_size_;
  double ndt_epsilon_;
  int ndt_max_iterations_;

  // --- 状态变量 ---
  std::mutex result_mtx_; 
  Eigen::Isometry3d result_t_; // T_Map_Odom
  Eigen::Isometry3d previous_result_t_; 
  bool is_initialized_; 
  rclcpp::Time last_scan_time_;

  // --- 数据对象 ---
  std::shared_ptr<SCManager> scManager_; // PIMPL 指针
  Eigen::MatrixXd matrix_poses_; 
  int file_count_ = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_for_init_; 

  // small_gicp 对象
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>> register_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
