/**
 * @file xchu_mapping.cpp
 * @author XCHU (2022087641@qq.com)
 * @brief  基于ndt的里程计
 *         imu部分暂时效果不好，不建议打开
 * @version 1.0
 * @date 2020-09-20
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <xchu_mapping/odom_node.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "xchu_mapping_node");
  ROS_INFO("\033[1;32m---->\033[0m XCHU Odometry Started.");

  LidarOdom mapping;
  std::thread odom_thread(&LidarOdom::Run, &mapping);
  //  ros::Rate rate(200);
  //  while (ros::ok()) {
  //    mapping.Run();
  //    ros::spinOnce();
  //    rate.sleep();
  //  }
  ros::spin();
  return 0;
}

LidarOdom::LidarOdom() : nh_("~") {
  // 初始化参数
  ParamInitial();

  current_points_pub = nh_.advertise<sensor_msgs::PointCloud2>("/current_points", 10);
  current_odom_pub = nh_.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

  points_sub = nh_.subscribe("/filtered_points", 10, &LidarOdom::PcCB, this);
  odom_sub = nh_.subscribe("/odom_raw", 50, &LidarOdom::OdomCB, this);// 编码器
  imu_sub = nh_.subscribe("/kitti/oxts/imu", 500, &LidarOdom::ImuCB, this);
  if (use_gps_) {
    gps_sub = nh_.subscribe<nav_msgs::Odometry>("/gps_odom", 20, &LidarOdom::GpsCB, this);
  }
}

void LidarOdom::ParamInitial() {
  nh_.param<float>("ndt_resolution", ndt_res, 1.0);
  nh_.param<double>("ndt_step_size", step_size, 0.1);
  nh_.param<double>("ndt_trans_eps", trans_eps, 0.01);
  nh_.param<int>("ndt_max_iter", max_iter, 30);
  nh_.param<double>("min_add_scan_shift", min_add_scan_shift, 0.5);
  nh_.param<int>("surround_search_num_", surround_search_num_, 20);
  nh_.param<double>("max_submap_size", max_localmap_size, 3);
  nh_.param<bool>("use_imu", _use_imu, false);
  nh_.param<bool>("use_odom", _use_odom, false);
  nh_.param<bool>("imu_upside_down", _imu_upside_down, false);
  nh_.param<bool>("incremental_voxel_update", _incremental_voxel_update, false);
  nh_.param<bool>("use_gps", use_gps_, false);
  nh_.param<int>("ndt_method_type", method_type_temp, 0);
  nh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "velo_link");
  nh_.param<std::string>("world_frame_id", world_frame_id_, "map");

  _method_type = static_cast<MethodType>(method_type_temp);
  if (_method_type == MethodType::use_pcl) {
    std::cout << ">> Use PCL NDT <<" << std::endl;
    pcl_ndt.setTransformationEpsilon(trans_eps);
    pcl_ndt.setStepSize(step_size);
    pcl_ndt.setResolution(ndt_res);
    pcl_ndt.setMaximumIterations(max_iter);
  } else if (_method_type == MethodType::use_cpu) {
    std::cout << ">> Use CPU NDT <<" << std::endl;
    cpu_ndt.setTransformationEpsilon(trans_eps);
    cpu_ndt.setStepSize(step_size);
    cpu_ndt.setResolution(ndt_res);
    cpu_ndt.setMaximumIterations(max_iter);
  } else if (_method_type == MethodType::use_omp) {
    std::cout << ">> Use OMP NDT <<" << std::endl;
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(omp_get_num_threads());  //  设置最大线程, 注意：需要引入头文件omp.h
    ndt->setTransformationEpsilon(trans_eps);
    ndt->setStepSize(step_size);
    ndt->setResolution(ndt_res);
    ndt->setMaximumIterations(max_iter);
    // 注意：此处设置ndt参数之后才能赋值给registration,否则后面无法使用getFinalScore函数！
    omp_ndt = ndt;
  } else {
    ROS_ERROR("Please Define _method_type to conduct NDT");
  }
  // 这里一般需要设定初始的雷达高度init_z
  nh_.param<double>("init_x", _tf_x, 0);
  nh_.param<double>("init_y", _tf_y, 0);
  nh_.param<double>("init_z", _tf_z, 0);
  nh_.param<double>("init_roll", _tf_roll, 0);
  nh_.param<double>("init_pitch", _tf_pitch, 0);
  nh_.param<double>("init_yaw", _tf_yaw, 0);
  Pose6D tl_pose(_tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw);
  tf_b2l = Pose6D2Matrix(tl_pose).cast<float>();
  tf_l2b = tf_b2l.inverse();

  downSizeFilterKeyframes.setLeafSize(filter_size, filter_size, filter_size); // 发布全局地图的采样size,设置小了导致系统卡顿
  downSizeFilterLocalmap.setLeafSize(filter_size * 2, filter_size * 2, filter_size * 2);
  downSizeFilterGlobalMap.setLeafSize(filter_size, filter_size, filter_size);

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  scan_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pc_target_.reset(new pcl::PointCloud<PointT>());
  transformed_scan_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  t_localizer_.setIdentity();
  t_base_link_.setIdentity();

  // 定义速度值 --包括实际速度值,和imu取到的速度值
  current_velocity_x = current_velocity_y = current_velocity_z = 0.0;
  current_velocity_imu_x = current_velocity_imu_y = current_velocity_imu_z = 0.0;
}

void LidarOdom::Run() {
  while (1) {
    if (_use_imu) {
      while (!cloud_queue_.empty() && !imu_queue_.empty()) {
        //align time stamp
        mutex_lock.lock();
        double time_diff = cloud_queue_.front()->header.stamp.toSec() - imu_queue_.front()->header.stamp.toSec();
        //      ROS_WARN("IMU CLOUD TIME DIFF %f", time_diff);
        if (!imu_queue_.empty() && imu_queue_.front()->header.stamp.toSec()
            < cloud_queue_.front()->header.stamp.toSec() - 0.5 * 0.1) {
          ROS_WARN(
              "odom_node: time stamp unaligned error and imu discarded, pls check your data; odom time %f, pc time %f",
              imu_queue_.front()->header.stamp.toSec(),
              cloud_queue_.front()->header.stamp.toSec());
          imu_queue_.pop();
          mutex_lock.unlock();
          continue;
        }
        if (!cloud_queue_.empty() && cloud_queue_.front()->header.stamp.toSec()
            < imu_queue_.front()->header.stamp.toSec() - 0.5 * 0.1) {
          ROS_WARN(
              "odom_node: time stamp unaligned error and imu discarded, pls check your data; odom time %f, pc time %f",
              imu_queue_.front()->header.stamp.toSec(),
              cloud_queue_.front()->header.stamp.toSec());
          cloud_queue_.pop();
          mutex_lock.unlock();
          continue;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in);
        ros::Time current_scan_time = (*cloud_queue_.front()).header.stamp;
        sensor_msgs::ImuConstPtr imu_msg = imu_queue_.front();
        imu_queue_.pop();
        cloud_queue_.pop();
        mutex_lock.unlock();

        // 1.滑窗localmap
        //      ExtractSurroundKeyframes();
        // 2.基于距离刷新
        //      ExtractSurroundKeyframesByDis();
        OdomEstimate(pointcloud_in, current_scan_time, imu_msg);
      }
    } else {
      while (!cloud_queue_.empty()) {
        mutex_lock.lock();
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_queue_.front(), *pointcloud_in);
        ros::Time current_scan_time = (*cloud_queue_.front()).header.stamp;
        cloud_queue_.pop();
        mutex_lock.unlock();

        // system_time = current_scan_time;
        // 等待GNSS初始化全局位姿, 只执行一次
        if (use_gps_) {
          if (!system_initialized_) {
            system_initialized_ = SystemInit(current_scan_time);
            ROS_WARN("Waiting for system initialized..");
            continue;
          }
        }

        // 匹配
        //       ExtractSurroundKeyframes();
        //      ExtractSurroundKeyframesByDis();
        OdomEstimate(pointcloud_in, current_scan_time, nullptr);
      }
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }

}

void LidarOdom::OdomEstimate(const pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_scan_ptr,
                             const ros::Time &current_scan_time, const sensor_msgs::ImuConstPtr &imu_msg) {
  ndt_start = ros::Time::now();
  if (filtered_scan_ptr->empty()) {
    ROS_ERROR("check your cloud...");
    return;
  }
  transformed_scan_ptr_->clear();
  if (initial_scan_loaded == 0 || cloud_keyframes_.empty()) {
    // 点云转换到车体坐标系, 这里可以采用gps进行初始化
    Eigen::Matrix4f init_matrix = Pose6D2Matrix(init_pose_).matrix().cast<float>();
    std::cout << "init_matrix: " << init_matrix << std::endl;

    pcl::transformPointCloud(*filtered_scan_ptr,
                             *transformed_scan_ptr_,
                             init_matrix * tf_l2b);  // tf_btol为初始变换矩阵
    localmap += *transformed_scan_ptr_;
    *pc_target_ += *transformed_scan_ptr_;
    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(pc_target_);
    else if (_method_type == MethodType::use_cpu)
      cpu_ndt.setInputTarget(pc_target_);
    else if (_method_type == MethodType::use_omp)
      omp_ndt->setInputTarget(pc_target_);
    ROS_INFO("new ndt target set");
    initial_scan_loaded = 1;
  }
  pc_target_.reset(new pcl::PointCloud<pcl::PointXYZI>(localmap));

  // 计算初始姿态，上一帧点云结果+两针之间的匀速运动估计,pitch和roll不变
  guess_pose = previous_pose + diff_pose;
  guess_pose.pitch = previous_pose.pitch;
  guess_pose.roll = previous_pose.roll;

  // 根据是否使用imu和odom,按照不同方式更新guess_pose(xyz,or/and rpy)
  Pose6D guess_pose_for_ndt;
  if (_use_imu && _use_odom) {
    ImuOdomCalc(current_scan_time);
    guess_pose_for_ndt = guess_pose_imu_odom;
  } else if (_use_imu && !_use_odom) {
    // 计算imu的相关信息， 速度以及rpy变化量
    CaculateAccAndVelo(*imu_msg);
    guess_pose_for_ndt = guess_pose_imu;
  } else if (!_use_imu && _use_odom) {
    OdomCalc(current_scan_time);
    guess_pose_for_ndt = guess_pose_odom;
  } else
    guess_pose_for_ndt = guess_pose;

  Eigen::Matrix4f init_guess = Pose6D2Matrix(guess_pose_for_ndt).cast<float>();

  // 用以保存ndt转换后的点云,align参数
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (_method_type == MethodType::use_pcl) {
    pcl_ndt.setInputSource(filtered_scan_ptr);
    pcl_ndt.align(*output_cloud, init_guess);  // pcl::aligin 需传入转换后的点云(容器),估计变换
    fitness_score = pcl_ndt.getFitnessScore();
    t_localizer_ = pcl_ndt.getFinalTransformation();  // t_localizer为ndt变换得到的最终变换矩阵(即source和target之间的变换)
    has_converged = pcl_ndt.hasConverged();
    final_num_iteration = pcl_ndt.getFinalNumIteration();
  } else if (_method_type == MethodType::use_cpu) {
    cpu_ndt.setInputSource(filtered_scan_ptr);
    cpu_ndt.align(*output_cloud, init_guess);
    fitness_score = cpu_ndt.getFitnessScore();
    t_localizer_ = cpu_ndt.getFinalTransformation();
    has_converged = cpu_ndt.hasConverged();
    final_num_iteration = cpu_ndt.getFinalNumIteration();
  } else if (_method_type == MethodType::use_omp) {
    omp_ndt->setInputSource(filtered_scan_ptr);
    omp_ndt->align(*output_cloud, init_guess);
    fitness_score = omp_ndt->getFitnessScore();
    t_localizer_ = omp_ndt->getFinalTransformation();
    has_converged = omp_ndt->hasConverged();
    final_num_iteration = omp_ndt->getFinalNumIteration();
  }
  ndt_end = ros::Time::now();

  // t_localizer是相对位姿,t_base_link对应的是全局位姿
  t_base_link_ = t_localizer_ * tf_l2b;
  pcl::transformPointCloud(*filtered_scan_ptr, *transformed_scan_ptr_, t_localizer_);   // 当前帧转换到全局地图上
  localizer_pose_ = Matrix2Pose6D(t_localizer_.cast<double>());
  current_pose = Matrix2Pose6D(t_base_link_.cast<double>());

  // current_pose 对应的是全局下的坐标!
  PointT this_pose_3d;
  this_pose_3d.x = current_pose.x;
  this_pose_3d.y = current_pose.y;
  this_pose_3d.z = current_pose.z;
  this_pose_3d.intensity = cloud_keyposes_3d_->points.size();  // 强度字段表示pose的index
  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyframes_.push_back(filtered_scan_ptr);
  cloud_keyposes_.push_back(t_base_link_);

  // 根据current和previous两帧之间的scantime,以及两帧之间的位置,计算两帧之间的变化
  double secs = (current_scan_time - previous_scan_time).toSec();
  diff_pose = current_pose - previous_pose;
  double diff = sqrt(diff_pose.x * diff_pose.x + diff_pose.y * diff_pose.y + diff_pose.z * diff_pose.z);

  // 用slam的结果修正imu速度
  current_velocity_x = diff_pose.x / secs;
  current_velocity_y = diff_pose.y / secs;
  current_velocity_z = diff_pose.z / secs;
  current_velocity_imu_x = current_velocity_x;
  current_velocity_imu_y = current_velocity_y;
  current_velocity_imu_z = current_velocity_z;

  // Update position and posture. current_pos -> previous_pos
  shift_dis = sqrt(pow(current_pose.x - previous_pose.x, 2.0) + pow(current_pose.y - previous_pose.y, 2.0));

  previous_pose = current_pose_imu = current_pose_odom = current_pose_imu_odom = current_pose;
  previous_scan_time = current_scan_time;
  offset_imu_pose = {0, 0, 0, 0, 0, 0};
  offset_odom_pose = {0, 0, 0, 0, 0, 0};
  offset_imu_odom_pose = {0, 0, 0, 0, 0, 0};

  // 更新localmap
  if (shift_dis >= min_add_scan_shift) {
    localmap_size += shift_dis;
    odom_size += shift_dis;

    // 这里的下采样网格大小将严重影响见图效果
    downSizeFilterLocalmap.setInputCloud(transformed_scan_ptr_);
    downSizeFilterLocalmap.filter(*transformed_scan_ptr_);
    localmap += *transformed_scan_ptr_; // localmap内的距离达到阈值就清空,并重新从0开始一帧一帧添加点云
    tmp_map += *transformed_scan_ptr_;

    // 注意:此时加入的target:map_ptr并不包括刚加入点云的transformed_scan_ptr
    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(pc_target_);
    else if (_method_type == MethodType::use_cpu) {
      if (_incremental_voxel_update)
        cpu_ndt.updateVoxelGrid(transformed_scan_ptr_);
      else
        cpu_ndt.setInputTarget(pc_target_);
    } else if (_method_type == MethodType::use_omp)
      omp_ndt->setInputTarget(pc_target_);
  }
  // 当局部地图内的距离大于阈值,则清空localmap
  if (localmap_size >= max_localmap_size) {
    localmap = tmp_map;
    tmp_map.clear();
    localmap_size = 0.0;
  }

  // 实时的点云也发布
  if (current_points_pub.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2::Ptr pointcloud_current_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*transformed_scan_ptr_, *pointcloud_current_ptr);
    pointcloud_current_ptr->header.frame_id = world_frame_id_;
    pointcloud_current_ptr->header.stamp = current_scan_time;
    current_points_pub.publish(*pointcloud_current_ptr);
  }
  if (current_odom_pub.getNumSubscribers() > 0) {
    Eigen::Quaternionf tmp_q(t_localizer_.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_scan_time;
    odom.header.frame_id = world_frame_id_;

    odom.pose.pose.position.x = t_base_link_(0, 3);
    odom.pose.pose.position.y = t_base_link_(1, 3);
    odom.pose.pose.position.z = t_base_link_(2, 3);

    odom.pose.pose.orientation.x = tmp_q.x();
    odom.pose.pose.orientation.y = tmp_q.y();
    odom.pose.pose.orientation.z = tmp_q.z();
    odom.pose.pose.orientation.w = tmp_q.w();

    odom.child_frame_id = lidar_frame_id_;
    odom.twist.twist.linear.x = current_velocity_x;
    odom.twist.twist.linear.y = current_velocity_y;
    odom.twist.twist.linear.z = current_velocity_z;
    current_odom_pub.publish(odom);

    // tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(t_base_link_(0, 3),
                                    t_base_link_(1, 3),
                                    t_base_link_(2, 3)));
    q.setW(tmp_q.w());
    q.setX(tmp_q.x());
    q.setY(tmp_q.y());
    q.setZ(tmp_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, current_scan_time, world_frame_id_, lidar_frame_id_));
  }

   std::cout << "*************************************************************************" << std::endl;
   std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
   std::cout << "Number of localmap points: " << pc_target_->size() << " points." << std::endl;
   std::cout << "Aligned Time: " << (ndt_end - ndt_start) * 1000 << " ms" << std::endl;
   std::cout << "Fitness score: " << fitness_score << std::endl;
   std::cout << "Number of iteration: " << final_num_iteration << std::endl;
   std::cout << "scan shift: " << shift_dis << std::endl;
   std::cout << "localmap shift: " << localmap_size << std::endl;
   std::cout << "global path: " << odom_size << std::endl;
}

void LidarOdom::ImuCB(const sensor_msgs::ImuConstPtr &msg) {
//  std::lock_guard<std::mutex> lock(imu_data_mutex);
//  if (_imu_upside_down)  // _imu_upside_down指示是否进行imu的正负变换
//    imuUpSideDown(msg);

  if (_use_imu) {
    mutex_lock.lock();
    imu_queue_.push(msg);
    mutex_lock.unlock();
  }
  /*const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  // 解析imu消息,获得rpy
  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = warpToPmPi(imu_roll);  // 调整,防止超过PI(180°)  --保持在±180°内
  imu_pitch = warpToPmPi(imu_pitch);
  imu_yaw = warpToPmPi(imu_yaw);

  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);
  //

  imu.header = input->header;
  imu.linear_acceleration.x = input->linear_acceleration.x;
  // imu.linear_acceleration.y = input->linear_acceleration.y;
  // imu.linear_acceleration.z = input->linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0) {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  } else {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;*/
}

void LidarOdom::OdomCB(const nav_msgs::OdometryConstPtr &msg) {
  if (_use_odom) {
    mutex_lock.lock();
    odom_queue_.push(msg);
    mutex_lock.unlock();
  }
}

void LidarOdom::PcCB(const sensor_msgs::PointCloud2ConstPtr &msg) {
  mutex_lock.lock();
  cloud_queue_.push(msg);
  mutex_lock.unlock();
}

void LidarOdom::GpsCB(const nav_msgs::OdometryConstPtr &msg) {
  if (use_gps_) {
    mutex_lock.lock();
    gps_deque_.push_back(*msg);
    mutex_lock.unlock();
  }
}

bool LidarOdom::SystemInit(const ros::Time &stamp) {
  // 以点云时间戳为系统时间
  if (use_gps_) {
    ROS_WARN(" gps_deque_ size : %f ", gps_deque_.size());
    bool gps_init = false;
    while (!gps_deque_.empty()) {
      mutex_lock.lock();
      double off_time = stamp.toSec() - gps_deque_.front().header.stamp.toSec();
      if (off_time > 0.5) {
        gps_deque_.pop_front();
        mutex_lock.unlock();
      } else if (off_time < -0.5) {
        ROS_ERROR("failed to algned gps and lidar time, %f", off_time);
        break;
      } else {
        nav_msgs::Odometry msg = gps_deque_.front();
        gps_deque_.pop_front();
        mutex_lock.unlock();
        ROS_WARN(" gps lidar off time: %f ", off_time);
        init_pose_.x = msg.pose.pose.position.x;
        init_pose_.y = msg.pose.pose.position.y;
        init_pose_.z = msg.pose.pose.position.z;

        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(msg.pose.pose.orientation.x,
                                     msg.pose.pose.orientation.y,
                                     msg.pose.pose.orientation.z,
                                     msg.pose.pose.orientation.w)).getRPY(roll, pitch, yaw);

        init_pose_.roll = roll;
        init_pose_.pitch = pitch;
        init_pose_.yaw = yaw;

        guess_pose = previous_pose = current_pose = init_pose_;
        gps_init = true;
        break;
      }
    }

    // 初始化状态
    if (gps_init) {
      ROS_WARN("System initialized with gnss...");
      return true;
    } else {
      ROS_ERROR("GPS initialized failed...");
      return false;
    }
  } else {
    guess_pose = previous_pose = current_pose = {0, 0, 0, 0, 0, 0};
    ROS_WARN("System initialized without gnss...");
    return true;
  }
}

void LidarOdom::ImuOdomCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();  // static声明的变量只会在第一次使用时被声明,因此不会被覆盖

  // imu信息处理,计算 -- imu只使用陀螺仪,即 只输出转角信息roll,pitch,yaw
  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu_odom.roll += diff_imu_roll;  // 更新current_pose_imu_odom相关,作为历史记录
  current_pose_imu_odom.pitch += diff_imu_pitch;
  current_pose_imu_odom.yaw += diff_imu_yaw;

  // odom信息处理,计算 -- xyz移动距离的计算,融合odom的速度(位移)信息和imu的转角信息
  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_imu_odom_pose.x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
  offset_imu_odom_pose.y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
  offset_imu_odom_pose.z += diff_distance * sin(-current_pose_imu_odom.pitch);

  offset_imu_odom_pose.roll += diff_imu_roll;
  offset_imu_odom_pose.pitch += diff_imu_pitch;
  offset_imu_odom_pose.yaw += diff_imu_yaw;

  // ==> 最终的目的是融合imu和odom输出一个guess_pose
  // 注:guess_pose是在previous_pose基础上叠加一个offset,包括xyz的和rpy的
  // xyz的offset需要融合imu的转角和odom的速度(位移)
  // rpy的offset直接采用imu的rpy偏差值
  guess_pose_imu_odom.x = previous_pose.x + offset_imu_odom_pose.x;
  guess_pose_imu_odom.y = previous_pose.y + offset_imu_odom_pose.y;
  guess_pose_imu_odom.z = previous_pose.z + offset_imu_odom_pose.z;
  guess_pose_imu_odom.roll = previous_pose.roll + offset_imu_odom_pose.roll;
  guess_pose_imu_odom.pitch = previous_pose.pitch + offset_imu_odom_pose.pitch;
  guess_pose_imu_odom.yaw = previous_pose.yaw + offset_imu_odom_pose.yaw;

  previous_time = current_time;
}

void LidarOdom::ImuCalc2(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  //current_pose_imu.roll += diff_imu_roll;
  //current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  // 对imu由于不平衡造成的补偿问题,在这里解决
  // start1
  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
      std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
      std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;
  // end1

  // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
  offset_imu_pose.x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_pose.y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_pose.z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;  // imu的速度值会通过slam进行修正,以避免累计误差
  current_velocity_imu_y += accY * diff_time;  // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
  current_velocity_imu_z += accZ * diff_time;    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

  offset_imu_pose.roll += diff_imu_roll;
  offset_imu_pose.pitch += diff_imu_pitch;
  offset_imu_pose.yaw += diff_imu_yaw;

  // pitch和roll方向的便宜不要加上
  guess_pose_imu = previous_pose + offset_imu_pose;
  guess_pose_imu.z = previous_pose.z;
  guess_pose_imu.roll = previous_pose.roll;
  guess_pose_imu.pitch = previous_pose.pitch;
  previous_time = current_time;
}

void LidarOdom::ImuCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  // 对imu由于不平衡造成的补偿问题,在这里解决
  // start1
  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
      std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
      std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;
  // end1

  // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
  offset_imu_pose.x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_pose.y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_pose.z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;  // imu的速度值会通过slam进行修正,以避免累计误差
  current_velocity_imu_y += accY * diff_time;  // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
  current_velocity_imu_z += accZ * diff_time;    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

  offset_imu_pose.roll += diff_imu_roll;
  offset_imu_pose.pitch += diff_imu_pitch;
  offset_imu_pose.yaw += diff_imu_yaw;

  guess_pose_imu = previous_pose + offset_imu_pose;
//  guess_pose_imu.x = previous_pose.x + offset_imu_pose.x;
//  guess_pose_imu.y = previous_pose.y + offset_imu_pose.y;
//  guess_pose_imu.z = previous_pose.z + offset_imu_pose.z;
//  guess_pose_imu.roll = previous_pose.roll + offset_imu_pose.roll;
//  guess_pose_imu.pitch = previous_pose.pitch + offset_imu_pose.pitch;
//  guess_pose_imu.yaw = previous_pose.yaw + offset_imu_pose.yaw;

  previous_time = current_time;
}

void LidarOdom::OdomCalc(ros::Time current_time) {
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_pose.x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_pose.y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_pose.z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_pose.roll += diff_odom_roll;
  offset_odom_pose.pitch += diff_odom_pitch;
  offset_odom_pose.yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose.x + offset_odom_pose.x;
  guess_pose_odom.y = previous_pose.y + offset_odom_pose.y;
  guess_pose_odom.z = previous_pose.z + offset_odom_pose.z;
  guess_pose_odom.roll = previous_pose.roll + offset_odom_pose.roll;
  guess_pose_odom.pitch = previous_pose.pitch + offset_odom_pose.pitch;
  guess_pose_odom.yaw = previous_pose.yaw + offset_odom_pose.yaw;

  previous_time = current_time;
}

void LidarOdom::imuUpSideDown(const sensor_msgs::Imu::Ptr input) {
  double input_roll, input_pitch, input_yaw;

  tf::Quaternion input_orientation;
  tf::quaternionMsgToTF(input->orientation, input_orientation);
  tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

  input->angular_velocity.x *= -1;
  input->angular_velocity.y *= -1;
  input->angular_velocity.z *= -1;

  input->linear_acceleration.x *= -1;
  input->linear_acceleration.y *= -1;
  input->linear_acceleration.z *= -1;

  input_roll *= -1;
  input_pitch *= -1;
  input_yaw *= -1;

//  input_yaw += M_PI/2;
  input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

void LidarOdom::CaculateAccAndVelo(const sensor_msgs::Imu &input) {
  const ros::Time current_time = input.header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  // 解析imu消息,获得rpy
  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input.orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  // imu的角度变化量，因为6轴测量的是相对变化量
  imu_roll = warpToPmPi(imu_roll);  // 调整,防止超过PI(180°)  --保持在±180°内
  imu_pitch = warpToPmPi(imu_pitch);
  imu_yaw = warpToPmPi(imu_yaw);

  // 计算可用的角度offset
  static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_pose_imu.roll);
  const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_pose_imu.pitch);
  const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_pose_imu.yaw);

  // 计算和之前的加速度和角速度
  imu.header = input.header;
  imu.linear_acceleration.x = input.linear_acceleration.x;
  // imu.linear_acceleration.y = input.linear_acceleration.y;
  // imu.linear_acceleration.z = input.linear_acceleration.z;
  imu.linear_acceleration.y = 0;
  imu.linear_acceleration.z = 0;

  if (diff_time != 0) {
    imu.angular_velocity.x = diff_imu_roll / diff_time;
    imu.angular_velocity.y = diff_imu_pitch / diff_time;
    imu.angular_velocity.z = diff_imu_yaw / diff_time;
  } else {
    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;
  }

  //ImuCalc(input.header.stamp);
  current_pose_imu.roll += diff_imu_roll;
  current_pose_imu.pitch += diff_imu_pitch;
  current_pose_imu.yaw += diff_imu_yaw;

  // 对imu由于不平衡造成的补偿问题,在这里解决
  // start1
  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(current_pose_imu.roll) * imu.linear_acceleration.y -
      std::sin(current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(current_pose_imu.roll) * imu.linear_acceleration.y +
      std::cos(current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(current_pose_imu.pitch) * accZ1 + std::cos(current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(current_pose_imu.pitch) * accZ1 - std::sin(current_pose_imu.pitch) * accX1;

  double accX = std::cos(current_pose_imu.yaw) * accX2 - std::sin(current_pose_imu.yaw) * accY2;
  double accY = std::sin(current_pose_imu.yaw) * accX2 + std::cos(current_pose_imu.yaw) * accY2;
  double accZ = accZ2;
  // end1

  // imu计算xyz方向上的偏移,初始速度用的imu_x为slam计算获得,后面再加上考虑加速度以及时间参数,获得较为准确的距离偏移
  offset_imu_pose.x += current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  offset_imu_pose.y += current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  offset_imu_pose.z += current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  current_velocity_imu_x += accX * diff_time;  // imu的速度值会通过slam进行修正,以避免累计误差
  current_velocity_imu_y += accY * diff_time;  // 或者说imu计算时所用的速度并不是用imu得到的,而是用slam得到的
  current_velocity_imu_z += accZ * diff_time;    // imu所提供的参数,主要用来计算角度上的偏移,以及加速度导致的距离上的偏移!@

  offset_imu_pose.roll += diff_imu_roll;
  offset_imu_pose.pitch += diff_imu_pitch;
  offset_imu_pose.yaw += diff_imu_yaw;

  guess_pose_imu = previous_pose + offset_imu_pose;
  //guess_pose_imu.roll = previous_pose.roll;
  //guess_pose_imu.pitch = previous_pose.pitch;

  previous_time = current_time;
  previous_pose_imu.roll = imu_roll;
  previous_pose_imu.pitch = imu_pitch;
  previous_pose_imu.yaw = imu_yaw;
}

void LidarOdom::odom_info(const nav_msgs::Odometry &input) {
  OdomCalc(input.header.stamp);
}

void LidarOdom::ExtractSurroundKeyframes() {
  if (cloud_keyframes_.empty()) {
    return;
  }
  // recent_keyframes_存储localmap
  bool target_updated = false;
  if (recent_keyframes_.size() < surround_search_num_) {
    // 关键帧数量不够的时候，往recent_keyframes_里面加点云即可
    recent_keyframes_.clear();
    for (int i = cloud_keyposes_3d_->points.size() - 1; i >= 0; --i) {
      int this_key_id = int(cloud_keyposes_3d_->points[i].intensity);
      pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*cloud_keyframes_[this_key_id],
                               *tf_cloud,
                               cloud_keyposes_[this_key_id]);   // 当前帧转换到全局地图上
      downSizeFilterLocalmap.setInputCloud(tf_cloud);
      downSizeFilterLocalmap.filter(*tf_cloud);
      recent_keyframes_.push_back(tf_cloud);
      if (recent_keyframes_.size() >= surround_search_num_) {
        break;
      }
    }
    target_updated = true;
  } else {
    // 关键帧数量够的时候,只去掉最老镇点云，加入新的一帧
    static int latest_frame_id = cloud_keyframes_.size() - 1;
    if (latest_frame_id != cloud_keyframes_.size() - 1) {
      latest_frame_id = cloud_keyframes_.size() - 1;
      recent_keyframes_.pop_back();
      pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*cloud_keyframes_[latest_frame_id],
                               *tf_cloud,
                               cloud_keyposes_[latest_frame_id]);   // 当前帧转换到全局地图上
      downSizeFilterLocalmap.setInputCloud(tf_cloud);
      downSizeFilterLocalmap.filter(*tf_cloud);
      recent_keyframes_.push_front(tf_cloud);
      target_updated = true;
    }
  }

  // localmap有更新的话，将recent_keyframes_中的点云取出来装到pc_target_中，作为点云匹配的target cloud
  if (target_updated) {
    pc_target_->clear();
    for (auto keyframe : recent_keyframes_) {
      *pc_target_ += *keyframe;
    }
    downSizeFilterKeyframes.setInputCloud(pc_target_);
    downSizeFilterKeyframes.filter(*pc_target_);

    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(pc_target_);
    else if (_method_type == MethodType::use_cpu)
      cpu_ndt.setInputTarget(pc_target_);
    else if (_method_type == MethodType::use_omp)
      omp_ndt->setInputTarget(pc_target_);
    //ROS_INFO("new ndt target set");
  }

/* if (pub_recent_keyframes_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pc_target_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub_recent_keyframes_.publish(msg);
  }*/
}

void LidarOdom::ExtractSurroundKeyframesByDis() {
  if (cloud_keyframes_.empty()) {
    return;
  }
  bool target_updated = false;
  if (shift_dis >= min_add_scan_shift) {
    localmap_size += shift_dis;
    odom_size += shift_dis;
    // 这里的下采样网格大小将影响见图效果
    downSizeFilterLocalmap.setInputCloud(transformed_scan_ptr_);
    downSizeFilterLocalmap.filter(*transformed_scan_ptr_);

    recent_keyframes_.push_back(transformed_scan_ptr_);
    // localmap += *transformed_scan_ptr; // localmap内的距离达到阈值就清空,并重新从0开始一帧一帧添加点云
    //    tmp_map += *transformed_scan_ptr;
    target_updated = true;
  }

  if (target_updated) {
    pc_target_->clear();
    for (auto keyframe : recent_keyframes_) {
      *pc_target_ += *keyframe;
    }
    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(pc_target_);
    else if (_method_type == MethodType::use_cpu) {
      if (_incremental_voxel_update)
        cpu_ndt.updateVoxelGrid(transformed_scan_ptr_);
      else
        cpu_ndt.setInputTarget(pc_target_);
    } else if (_method_type == MethodType::use_omp)
      omp_ndt->setInputTarget(pc_target_);
    //ROS_INFO("new ndt target set");
  }

  if (localmap_size >= max_localmap_size) {
    pc_target_->clear();
    for (auto keyframe : recent_keyframes_) {
      *pc_target_ += *keyframe;
    }
    if (_method_type == MethodType::use_pcl)
      pcl_ndt.setInputTarget(pc_target_);
    else if (_method_type == MethodType::use_cpu) {
      if (_incremental_voxel_update)
        cpu_ndt.updateVoxelGrid(transformed_scan_ptr_);
      else
        cpu_ndt.setInputTarget(pc_target_);
    } else if (_method_type == MethodType::use_omp)
      omp_ndt->setInputTarget(pc_target_);

    localmap_size = 0.0;
    recent_keyframes_.clear();
  }
}
