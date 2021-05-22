

#ifndef NDT_MAPPING_LIDARMAPPING_H
#define NDT_MAPPING_LIDARMAPPING_H

#define OUTPUT  // define OUTPUT if you want to log postition.txt

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <deque>
#include <mutex>
#include <queue>
#include <thread>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pclomp/ndt_omp.h>
#include "omp.h"
#include "xchu_mapping/common.h"

enum class MethodType {
  use_pcl = 0,
  use_cpu = 1,
  use_gpu = 2,
  use_omp = 3,
};

class LidarOdom {
 private:
  Pose6D previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom;
  Pose6D current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom;
  Pose6D previous_pose_imu, previous_pose_odom, previous_pose_imu_odom; // imu的变化量
  Pose6D diff_pose_imu, diff_pose_odom, diff_pose_imu_odom; // 单个传感器的变化量
  Pose6D ndt_pose, localizer_pose;
  // 定义各种差异值(两次采集数据之间的差异,包括点云位置差异,imu差异,odom差异,imu-odom差异)
  Pose6D diff_pose, offset_imu_pose, offset_odom_pose, offset_imu_odom_pose;

  // 定义速度值 --包括实际速度值,和imu取到的速度值
  double current_velocity_x, current_velocity_y, current_velocity_z;
  double current_velocity_imu_x, current_velocity_imu_y, current_velocity_imu_z;

  //  ros::Time current_scan_time;
  ros::Time previous_scan_time, previous_imu_time;
  ros::Time curr_imu_time;

  // 定义Publisher
  ros::Publisher current_odom_pub, odom_pose_pub, current_points_pub, ndt_stat_pub;  // TODO:这是个啥发布????
  ros::Subscriber points_sub, imu_sub, odom_sub;

  geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;
  std_msgs::Bool ndt_stat_msg;  // 确认是否是ndt的第一帧图像 bool类型
  sensor_msgs::Imu imu;
  nav_msgs::Odometry odom;

  MethodType _method_type;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> pcl_ndt;
  cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> cpu_ndt;  // cpu方式
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr omp_ndt;

  // 设置变量,用以接受ndt配准后的参数
  double fitness_score;
  bool has_converged;
  int final_num_iteration;

  // Default values  // 公共ndt参数设置
  int max_iter;        // Maximum iterations
  float ndt_res;      // Resolution
  double step_size;   // Step size
  double trans_eps;  // Transformation epsilon

  pcl::PointCloud<pcl::PointXYZI> localmap, globalmap, tmp_map;

#ifdef CUDA_FOUND
  gpu::GNormalDistributionsTransform gpu_ndt;
   std::shared_ptr<gpu::GNormalDistributionsTransform> gpu_ndt = std::make_shared<gpu::GNormalDistributionsTransform>();
#endif

  double min_add_scan_shift;  // 定义将点云添加到locaMap里的最小间隔值  --应该是添加到localMap吧??
  int initial_scan_loaded = 0;
  int surround_search_num_ = 0;

  bool _incremental_voxel_update = false;
  ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, ndt_start, ndt_end,
      t5_start, t5_end;
  ros::Duration d_callback, d1, d2, d3, d4, d5;
  int submap_num = 0;
  double localmap_size = 0.0;
  double max_localmap_size, odom_size;
  double shift_dis = 0.0;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_b2l, tf_l2b;

  bool _use_imu = false;
  bool _use_odom = false;
  bool _imu_upside_down = false;  // 用以解决坐标系方向(正负变换)问题 (比如x变更为-x等)
  int method_type_temp = 0;

  // mutex
  std::mutex mutex_lock;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
  std::queue<sensor_msgs::ImuConstPtr> imu_queue_;
  std::queue<nav_msgs::OdometryConstPtr> odom_queue_;

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterGlobalMap;
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterKeyFrames;
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterLocalmap;

  pcl::PointCloud<PointT>::Ptr cloud_keyposes_3d_;
  std::vector<Eigen::Matrix4f> cloud_keyposes_;

  std::deque<pcl::PointCloud<PointT>::Ptr> recent_keyframes_;
  std::vector<pcl::PointCloud<PointT>::Ptr> cloud_keyframes_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr, pc_target_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr;

  Eigen::Matrix4f t_localizer;
  Eigen::Matrix4f t_base_link;

 public:
  ros::NodeHandle nh;

  LidarOdom();

  inline double warpToPm(double a_num, const double a_max) {
    if (a_num >= a_max) {
      a_num -= 2.0 * a_max;
    }
    return a_num;
  }

  inline double warpToPmPi(double a_angle_rad) {
    return warpToPm(a_angle_rad, M_PI);
  }

  inline double calcDiffForRadian(const double lhs_rad, const double rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
      diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
  }

  void Run();

  void OdomEstimate(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                    const ros::Time &current_scan_time,
                    const sensor_msgs::ImuConstPtr &msg);

  void ParamInitial();

  void ImuCB(const sensor_msgs::ImuConstPtr &msg);

  void OdomCB(const nav_msgs::OdometryConstPtr &msg);

  void PcCB(const sensor_msgs::PointCloud2ConstPtr &msg);

  void ImuOdomCalc(ros::Time current_time);

  void ImuCalc(ros::Time current_time);

  void ImuCalc2(ros::Time current_time);

  void OdomCalc(ros::Time current_time);

  void imuUpSideDown(const sensor_msgs::Imu::Ptr input);

  void odom_info(const nav_msgs::Odometry &input);

  void CaculateAccAndVelo(const sensor_msgs::Imu &input);

  void ExtractSurroundKeyframes();

  void ExtractSurroundKeyframesByDis();
};

#endif
