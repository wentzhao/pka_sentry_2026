// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

// Small GICP headers
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "small_gicp/registration/registration_helper.hpp"

// NDT OMP header
#include <pclomp/ndt_omp.h>
#include <chrono> // For std::chrono

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  // --- Parameters ---
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  // NDT Params
  this->declare_parameter("ndt_resolution", 1.0);
  this->declare_parameter("ndt_step_size", 0.1);
  this->declare_parameter("ndt_epsilon", 0.01);
  
  // [保留] Debug 开关
  this->declare_parameter("debug", false);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  
  this->get_parameter("ndt_resolution", ndt_resolution_);
  this->get_parameter("ndt_step_size", ndt_step_size_);
  this->get_parameter("ndt_epsilon", ndt_epsilon_);
  
  this->get_parameter("debug", debug_);

  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  target_cloud_xyz_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // 加载地图
  loadGlobalMap(prior_pcd_file_);

  // Downsample & Prepare GICP Target
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // 防止 while(true) 死循环导致节点无法启动
  Eigen::Affine3d odom_to_lidar_odom = Eigen::Affine3d::Identity();
  bool tf_found = false;
  
  for(int i=0; i<5; i++) { // 尝试 5 次，每次 1 秒
      try {
        auto tf_stamped = tf_buffer_->lookupTransform(
          base_frame_, lidar_frame_, this->now(), std::chrono::seconds(1)); 
        odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
        
        RCLCPP_INFO_STREAM(
          this->get_logger(), "odom_to_lidar_odom: translation = "
                                << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                                << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
        tf_found = true;
        break;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed (%d/5): %s", i+1, ex.what());
      }
  }
  
  if (!tf_found) {
      RCLCPP_ERROR(this->get_logger(), "FAILED to find TF base->lidar after 5 seconds! Map might be misaligned!");
  }

  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);

  double ndt_leaf = std::max<double>(global_leaf_size_, 0.5);
  target_cloud_xyz_ = small_gicp::voxelgrid_sampling_omp<
      pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>(
      *global_map_, ndt_leaf);
  RCLCPP_INFO(this->get_logger(), "NDT target prepared. Points: %zu", target_cloud_xyz_->points.size());
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

// void SmallGicpRelocalizationNode::performRegistration()
// {
//   if (accumulated_cloud_->empty()) {
//     if (debug_) {
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No accumulated points to process.");
//     }
//     return;
//   }
  
//   if (!target_ || target_->empty()) return;

//   // --- 0. Preprocessing ---
//   auto source_xyz = small_gicp::voxelgrid_sampling_omp<
//     pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>(
//     *accumulated_cloud_, registered_leaf_size_);

//   if (source_xyz->empty()) return;

//   // --- 1. Coarse Registration: NDT ---
//   pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_omp;
//   ndt_omp.setResolution(ndt_resolution_);
//   ndt_omp.setTransformationEpsilon(ndt_epsilon_);
//   ndt_omp.setStepSize(ndt_step_size_);
//   ndt_omp.setNumThreads(num_threads_);
//   ndt_omp.setNeighborhoodSearchMethod(pclomp::KDTREE); 
//   ndt_omp.setInputSource(source_xyz);
//   ndt_omp.setInputTarget(target_cloud_xyz_); 

//   pcl::PointCloud<pcl::PointXYZ> unused_result;
//   Eigen::Matrix4f init_guess = previous_result_t_.matrix().cast<float>();
//   ndt_omp.align(unused_result, init_guess);

//   Eigen::Isometry3d ndt_result = Eigen::Isometry3d::Identity();
//   bool ndt_converged = ndt_omp.hasConverged();
//   double ndt_score = ndt_omp.getFitnessScore();

//   if (ndt_converged) {
//       ndt_result.matrix() = ndt_omp.getFinalTransformation().cast<double>();
//   } else {
//       // 只有 debug 模式才警告 NDT 失败，避免日志污染
//       if (debug_) {
//           RCLCPP_WARN(this->get_logger(), "NDT did not converge, using previous guess.");
//       }
//       ndt_result = previous_result_t_;
//   }

//   // --- 策略优化 ---
//   if (ndt_converged && ndt_score < 0.1) {
//       result_t_ = previous_result_t_ = ndt_result;
      
//       // [DEBUG] 只有在 debug 模式下打印完美匹配信息
//       if (debug_) {
//           RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
//             "NDT Perfect Match (Score: %.4f). Skipping GICP.", ndt_score);
//       }
//       accumulated_cloud_->clear();
//       return; 
//   }

//   // --- 2. Fine Registration: Small GICP ---
//   source_ = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
//   source_->resize(source_xyz->size());
//   #pragma omp parallel for num_threads(num_threads_)
//   for(size_t i=0; i<source_xyz->size(); i++) {
//       source_->points[i].x = source_xyz->points[i].x;
//       source_->points[i].y = source_xyz->points[i].y;
//       source_->points[i].z = source_xyz->points[i].z;
//   }
//   small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
//   source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
//     source_, small_gicp::KdTreeBuilderOMP(num_threads_));

//   if (!source_ || !source_tree_) return;

//   register_->reduction.num_threads = num_threads_;
//   if (!ndt_converged) {
//       register_->rejector.max_dist_sq = max_dist_sq_ * 5.0; 
//   } else {
//       register_->rejector.max_dist_sq = max_dist_sq_;
//   }
//   register_->optimizer.max_iterations = 20;

//   auto result = register_->align(*target_, *source_, *target_tree_, ndt_result);

//   if (result.converged) {
//     result_t_ = previous_result_t_ = result.T_target_source;
    
//     // [DEBUG] 只有在 debug 模式下打印 GICP 成功信息
//     if (debug_) {
//         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//             "GICP Success. Error: %.4f (NDT Score: %.4f)", result.error, ndt_score);
//     }
//   } else {
//     // 降级处理
//     if (ndt_converged && ndt_score < 0.5) {
//         result_t_ = previous_result_t_ = ndt_result;
        
//         // [DEBUG] 降级信息
//         if (debug_) {
//             RCLCPP_INFO(this->get_logger(), 
//                 "GICP failed (Err: %.2f) but NDT valid (Score: %.4f). Used NDT.", 
//                 result.error, ndt_score);
//         }
//     } else {
//         // [WARNING] 严重错误始终打印
//         RCLCPP_WARN(this->get_logger(), "Registration failed completely. GICP Err: %.2f", result.error);
//     }
//   }
//   accumulated_cloud_->clear();
// }

void SmallGicpRelocalizationNode::performRegistration()
{
  // 1. 检查是否有累积的点云
  if (accumulated_cloud_->empty()) {
    if (debug_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No accumulated points to process.");
    }
    return;
  }
  
  // 2. 检查目标地图是否加载
  if (!target_ || target_->empty()) return;

  // --- 0. 预处理 (Preprocessing) ---
  // 对当前扫描点云进行降采样，得到 XYZ 格式
  auto source_xyz = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>(
    *accumulated_cloud_, registered_leaf_size_);

  if (source_xyz->empty()) return;

  // --- 1. 粗配准: NDT (Coarse Registration) ---
  // 参考 sc_liorf_localization，先用 NDT 将点云拉到大致位置
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_omp;
  ndt_omp.setResolution(ndt_resolution_);
  ndt_omp.setTransformationEpsilon(ndt_epsilon_);
  ndt_omp.setStepSize(ndt_step_size_);
  ndt_omp.setNumThreads(num_threads_);
  ndt_omp.setNeighborhoodSearchMethod(pclomp::KDTREE); 
  
  ndt_omp.setInputSource(source_xyz);
  ndt_omp.setInputTarget(target_cloud_xyz_); 

  pcl::PointCloud<pcl::PointXYZ> unused_result;
  Eigen::Matrix4f init_guess = previous_result_t_.matrix().cast<float>();
  
  // 执行 NDT
  ndt_omp.align(unused_result, init_guess);

  Eigen::Isometry3d ndt_result = Eigen::Isometry3d::Identity();
  bool ndt_converged = ndt_omp.hasConverged();
  double ndt_score = ndt_omp.getFitnessScore();

  if (ndt_converged) {
      ndt_result.matrix() = ndt_omp.getFinalTransformation().cast<double>();
  } else {
      if (debug_) {
          RCLCPP_WARN(this->get_logger(), "NDT did not converge, using previous guess.");
      }
      ndt_result = previous_result_t_;
  }

  // --- 策略优化: NDT 极优则跳过 GICP ---
  // 类似于 LIO-SAM 中如果匹配很好就不需要过多的优化
  if (ndt_converged && ndt_score < 0.1) {
      result_t_ = previous_result_t_ = ndt_result;
      
      if (debug_) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "NDT Perfect Match (Score: %.4f). Skipping GICP.", ndt_score);
      }
      accumulated_cloud_->clear();
      return; 
  }

  // --- 2. 精配准: Small GICP (Fine Registration) ---
  
  // 转换格式为 PointCovariance
  source_ = std::make_shared<pcl::PointCloud<pcl::PointCovariance>>();
  source_->resize(source_xyz->size());
  
  // 并行填充数据
  #pragma omp parallel for num_threads(num_threads_)
  for(size_t i=0; i<source_xyz->size(); i++) {
      source_->points[i].x = source_xyz->points[i].x;
      source_->points[i].y = source_xyz->points[i].y;
      source_->points[i].z = source_xyz->points[i].z;
  }
  
  // 计算 Source 的协方差 (Local Plane Features)
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  
  // [优化] 移除 source_tree_ 的构建
  // Small GICP 的 align 函数只需要 Target 的 Tree 来做最近邻搜索，Source 不需要建树。
  // 这可以节省建树的时间。

  if (!source_) return;

  register_->reduction.num_threads = num_threads_;
  
  // 动态调整搜索范围
  // 如果 NDT 没收敛，说明初值可能不好，扩大 GICP 的捕获范围
  if (!ndt_converged) {
      register_->rejector.max_dist_sq = max_dist_sq_ * 5.0; 
  } else {
      register_->rejector.max_dist_sq = max_dist_sq_;
  }
  
  register_->optimizer.max_iterations = 20;

  // 执行 GICP，使用 NDT 结果作为初值
  // 注意：这里只传 target_tree_，不需要 source_tree
  auto result = register_->align(*target_, *source_, *target_tree_, ndt_result);

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
    
    if (debug_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "GICP Success. Error: %.4f (NDT Score: %.4f)", result.error, ndt_score);
    }
  } else {
    // --- 降级处理 (Fallback) ---
    // 如果 GICP 发散 (可能是因为环境特征退化)，但 NDT 结果尚可，则回退使用 NDT
    if (ndt_converged && ndt_score < 0.5) {
        result_t_ = previous_result_t_ = ndt_result;
        
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), 
                "GICP failed (Err: %.2f) but NDT valid (Score: %.4f). Used NDT.", 
                result.error, ndt_score);
        }
    } else {
        // 彻底失败，保持上一帧结果
        RCLCPP_WARN(this->get_logger(), "Registration failed completely. GICP Err: %.2f", result.error);
    }
  }
  
  accumulated_cloud_->clear();
}


void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) return;
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;
  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());
  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();
  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received initial pose...");
  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();

  try {
    auto transform = tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;
    previous_result_t_ = result_t_ = map_to_odom;

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform initial pose: %s", ex.what());
  }
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
