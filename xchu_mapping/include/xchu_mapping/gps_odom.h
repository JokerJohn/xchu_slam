/**
* @Program: Project
* @Description: gps_node
* @Author: Xiangcheng Hu
* @Create: 2020/11/26
* @Copyright: [2020] <Copyright hxc@2022087641@qq.com>
**/
#ifndef SRC_XCHU_SLAM_SRC_GPS_ODOM_H_
#define SRC_XCHU_SLAM_SRC_GPS_ODOM_H_

#include <mutex>
#include <queue>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <gps_tools/gpsTools.h>

#define DEG_TO_RAD 0.01745329252

class GNSSOdom {
 public:
  GNSSOdom();

  /**
   * gps odom主流程
   */
  void Run();

 private:
  /**
   * IMU callback
   * @param msg
   */
  void ImuCB(const sensor_msgs::ImuConstPtr &msg);

  /**
   * GNSS callback:会将gnss转换为xyz平面坐标
   * @param msg
   */
  void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg);

  /**
   * IMU姿态补偿
   * @param input
   */
  void imuUpsideDown(const sensor_msgs::Imu::Ptr &input);

  static inline double deg2rad(const double &deg) {
    return deg * DEG_TO_RAD;
  };

  static inline double wrapToPmPi(double a_angle_rad) {
    if (a_angle_rad >= M_PI) {
      a_angle_rad -= 2.0 * M_PI;
    }
    return a_angle_rad;
  }

 public:
  gpsTools gtools;
  double origin_latitude = 0.0, origin_longitude = 0.0, origin_altitude = 0.0;
  Eigen::Vector3d init_pos_;
  double origin_east = 0.0, origin_north = 0.0, origin_height;
// imu到lidar的转换
  Eigen::Matrix4d T_imu2velo = Eigen::Matrix4d::Identity();
  Eigen::Vector3d pos;
  Eigen::Matrix3d rot;

  ros::NodeHandle nh_;
  ros::Publisher gps_odom_pub_;
  ros::Subscriber imu_sub_, gps_sub_;
  std::string imu_topic, gps_topic, world_frame_id_;

  // imu和gps队列
  std::mutex mutex_lock;
  std::deque<sensor_msgs::ImuConstPtr> imuBuf;
  std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

  bool init_xyz = false;
  bool init_insxyz = false;
  bool init_utm = false;
  bool use_localmap = false;
};

#endif //SRC_XCHU_SLAM_SRC_GPS_ODOM_H_
