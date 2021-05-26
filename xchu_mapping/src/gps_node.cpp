/**
* @Program: Xchu slam
* @Description: GNSS Odometry
* @Author:  xchu
* @Create: 2020/11/26
* @Copyright: [2020] <Copyright 2022087641@qq.com>
**/

#include "xchu_mapping/gps_odom.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_node");

  GNSSOdom gps_node;
  std::thread gps_thread(&GNSSOdom::Run, &gps_node);

/*  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
    gps_node.run();
  }*/

  ros::spin();
  return 0;
}

void GNSSOdom::Run() {
  while (1) {
    if (!init_xyz) {
      //ROS_WARN("PLease init lla first!!!");
      continue;
    }
    while (!gpsBuf.empty() && !imuBuf.empty()) {
      ros::Time gps_stamp = gpsBuf.front()->header.stamp;
      bool imu_type = false;
      auto imu_iter = imuBuf.begin();
      for (imu_iter; imu_iter != imuBuf.end(); imu_iter++) {
        if (gps_stamp < (*imu_iter)->header.stamp) {
          break;
        }
        //      imu_msg.linear_acceleration = (*imu_iter)->linear_acceleration;
        //      imu_msg.angular_velocity = (*imu_iter)->angular_velocity;
        //      imu_msg.orientation = (*imu_iter)->orientation;
        //      imu_msg.orientation_covariance = (*imu_iter)->orientation_covariance;
        //      imu_msg.linear_acceleration_covariance = (*imu_iter)->linear_acceleration_covariance;
        //      imu_msg.angular_velocity_covariance = (*imu_iter)->angular_velocity_covariance;
        //      imu_msg.header.stamp = (*imu_iter)->header.stamp;
        imu_type = true;
      }
      mutex_lock.lock();
      imuBuf.erase(imuBuf.begin(), imu_iter);
      sensor_msgs::NavSatFixConstPtr gps_msg = gpsBuf.front();
      sensor_msgs::Imu imu_msg = *(imuBuf.front());
      gpsBuf.pop_front();
      mutex_lock.unlock();

      // 检验时间戳是否一致
      double imu_time = imu_msg.header.stamp.toSec();
      double gps_time = gps_msg->header.stamp.toSec();
      double off_time = gps_time - imu_time;
      if (off_time < 0.1 && off_time > -0.1) {
        //ROS_WARN("off set time: %f ", off_time);
      } else {
        ROS_ERROR("Time aligned failed....");
      }

      //  convert  LLA to XYZ
      Eigen::Vector3d lla = gtools.GpsMsg2Eigen(*gps_msg);
      Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
      Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
      //ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));

      // gps坐标转换到lidar系下 kitti的gps和imu安装在一起，所以对imu和lidar进行标定即可
      Eigen::Vector3d calib_enu = rot * enu + pos;
      //      std::cout << "pose bef and aft: " << enu(0) << ", " << enu(1) << ", " << enu(2) << std::endl;
      //      std::cout << "pose bef and aft: " << calib_enu(0) << ", " << calib_enu(1) << ", " << calib_enu(2) << std::endl;

      // pub odom
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = gps_msg->header.stamp;
      odom_msg.header.frame_id = world_frame_id_;
      odom_msg.child_frame_id = "gps";

      // ----------------- 1. use utm -----------------------
      //        odom_msg.pose.pose.position.x = utm_x - origin_utm_x;
      //        odom_msg.pose.pose.position.y = utm_y - origin_utm_y;
      //        odom_msg.pose.pose.position.z = msg->altitude - origin_al;

      // ----------------- 2. use enu -----------------------
      odom_msg.pose.pose.position.x = calib_enu(0);
      odom_msg.pose.pose.position.y = calib_enu(1);
      odom_msg.pose.pose.position.z = calib_enu(2);
      odom_msg.pose.covariance[0] = gps_msg->position_covariance[0];
      odom_msg.pose.covariance[7] = gps_msg->position_covariance[4];
      odom_msg.pose.covariance[14] = gps_msg->position_covariance[8];

      if (imu_type) {
        odom_msg.pose.pose.orientation = imu_msg.orientation;
        odom_msg.pose.covariance[21] = imu_msg.orientation_covariance[0];
        odom_msg.pose.covariance[22] = imu_msg.orientation_covariance[1];
        odom_msg.pose.covariance[23] = imu_msg.orientation_covariance[2];
        odom_msg.pose.covariance[27] = imu_msg.orientation_covariance[3];
        odom_msg.pose.covariance[28] = imu_msg.orientation_covariance[4];
        odom_msg.pose.covariance[29] = imu_msg.orientation_covariance[5];
        odom_msg.pose.covariance[33] = imu_msg.orientation_covariance[6];
        odom_msg.pose.covariance[34] = imu_msg.orientation_covariance[7];
        odom_msg.pose.covariance[35] = imu_msg.orientation_covariance[8];

        odom_msg.twist.twist.linear = imu_msg.linear_acceleration;
        odom_msg.twist.covariance[0] = imu_msg.linear_acceleration_covariance[0];
        odom_msg.twist.covariance[7] = imu_msg.linear_acceleration_covariance[4];
        odom_msg.twist.covariance[14] = imu_msg.linear_acceleration_covariance[8];

        odom_msg.twist.twist.angular = imu_msg.angular_velocity;
        odom_msg.twist.covariance[21] = imu_msg.angular_velocity_covariance[0];
        odom_msg.twist.covariance[28] = imu_msg.angular_velocity_covariance[4];
        odom_msg.twist.covariance[35] = imu_msg.angular_velocity_covariance[8];
        imu_type = false;
      }
      gps_odom_pub_.publish(odom_msg);
    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

GNSSOdom::GNSSOdom() : nh_("~") {
  nh_.param<std::string>("imu_topic", imu_topic, "/kitti/oxts/imu");
  nh_.param<std::string>("gps_topic", gps_topic, "/kitti/oxts/gps/fix");
  nh_.param<std::string>("world_frame_id", world_frame_id_, "map");
  nh_.param("use_localmap", use_localmap, false);  // 使用点云地图原点, 否则使用车辆运动的起点作为地图原点

  T_imu2velo << 9.999976e-01, 7.553071e-04, -2.035826e-03, -8.086759e-01,
      -7.854027e-04, 9.998898e-01, -1.482298e-02, 3.195559e-01,
      2.024406e-03, 1.482454e-02, 9.998881e-01, -7.997231e-01,
      0, 0, 0, 1;
  pos = T_imu2velo.block<3, 1>(0, 3).matrix();
  rot = T_imu2velo.block<3, 3>(0, 0).matrix();

  imu_sub_ = nh_.subscribe(imu_topic, 2000, &GNSSOdom::ImuCB, this);
  gps_sub_ = nh_.subscribe(gps_topic, 100, &GNSSOdom::GNSSCB, this);
  gps_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 4, false);
}

void GNSSOdom::ImuCB(const sensor_msgs::ImuConstPtr &msg) {
  mutex_lock.lock();
  imuBuf.push_back(msg);
  mutex_lock.unlock();
}

void GNSSOdom::GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
  if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
    ROS_ERROR("INS POS LLA NAN...");
    return;
  }
  // 设置世界坐标系原点为起始点
  if (!use_localmap && !init_xyz) {
    ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude, msg->altitude);
    gtools.lla_origin_ << msg->latitude, msg->longitude, msg->altitude;
    init_xyz = true;
  }
  mutex_lock.lock();
  gpsBuf.push_back(msg);
  mutex_lock.unlock();
}
