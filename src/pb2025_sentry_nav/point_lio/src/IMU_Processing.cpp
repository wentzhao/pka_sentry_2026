#include "IMU_Processing.h"

const bool time_list(PointType & x, PointType & y) { return (x.curvature < y.curvature); };

void ImuProcess::set_gyr_cov(const V3D & scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D & scaler) { cov_vel_scale = scaler; }

ImuProcess::ImuProcess()
: b_first_frame_(true), imu_need_init_(true), logger(rclcpp::get_logger("ImuProcess"))
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  after_imu_init_ = false;
  state_cov.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
  RCLCPP_WARN(logger, "reset ImuProcess");
  mean_acc = V3D(0, 0, 0.0);
  mean_gyr = V3D(0, 0, 0);
  imu_need_init_ = true;
  init_iter_num = 1;
  after_imu_init_ = false;

  time_last_scan = 0.0;
}

void ImuProcess::Set_init(Eigen::Vector3d & tmp_gravity, Eigen::Matrix3d & rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1), -gravity_(2), 0.0, gravity_(0), gravity_(1),
    -gravity_(0), 0.0;
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
  if (align_norm < 1e-6) {
    if (align_cos > 1e-6) {
      rot = Eye3d;
    } else {
      rot = -Eye3d;
    }
  } else {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

void ImuProcess::IMU_init(const MeasureGroup & meas, int & N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto & imu_acc = meas.imu.front()->linear_acceleration;
    const auto & gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto & imu : meas.imu) {
    const auto & imu_acc = imu->linear_acceleration;
    const auto & gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    N++;
  }
}

void ImuProcess::Process(const MeasureGroup & meas, PointCloudXYZI::Ptr cur_pcl_un_)
{
  if (imu_en) {
    if (meas.imu.empty()) return;

    if (imu_need_init_) {
      {
        /// The very first lidar frame
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;

      //   if (init_iter_num > MAX_INI_COUNT) {
      //     RCLCPP_INFO(logger, "IMU Initializing: %.1f %%", 100.0);
      //     imu_need_init_ = false;
      //     *cur_pcl_un_ = *(meas.lidar);
      //   }
      //   // *cur_pcl_un_ = *(meas.lidar);
      // }
      // return;
        // 当收集满数据（例如100帧）后
        if (init_iter_num > MAX_INI_COUNT) {
            
          // ================= [新增] 重力对齐核心代码 =================
          // 1. 计算平均加速度
          V3D avg_acc = mean_acc / double(init_iter_num);

          // 2. 计算方向向量
          // 加速度计测量的是支撑力，静止时指向"上" (Z轴)
          V3D acc_unit = avg_acc.normalized(); 
          V3D z_axis_world(0.0, 0.0, 1.0); // 世界坐标系的 Z 轴

          // 3. 计算旋转矩阵 R_wb (从 Body 到 World)
          // 我们需要一个旋转 R，使得 R * acc_unit = z_axis_world
          Eigen::Quaterniond q_init;
          q_init.setFromTwoVectors(acc_unit, z_axis_world);
          init_rot_ = q_init.toRotationMatrix();

          // 打印日志，确认计算结果
          RCLCPP_INFO(logger, "IMU Initializing Done!");
          RCLCPP_INFO(logger, "Avg Acc: %.3f, %.3f, %.3f", avg_acc.x(), avg_acc.y(), avg_acc.z());
          
          // 计算对应的欧拉角方便人眼检查 (Roll, Pitch, Yaw)
          V3D euler = init_rot_.eulerAngles(0, 1, 2) * 180 / M_PI;
          RCLCPP_INFO(logger, "Calculated Init RPY: %.2f, %.2f, %.2f", euler.x(), euler.y(), euler.z());
          // =========================================================

          imu_need_init_ = false;
          *cur_pcl_un_ = *(meas.lidar);
        }
      }
      return;
    }
    if (!after_imu_init_) after_imu_init_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  } else {
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}