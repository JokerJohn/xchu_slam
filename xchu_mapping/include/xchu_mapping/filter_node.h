
#ifndef SRC_XCHU_MAPPING_SRC_POINTS_FILTER_NODE_H_
#define SRC_XCHU_MAPPING_SRC_POINTS_FILTER_NODE_H_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <thread>
#include <mutex>
#include <queue>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp> /*找不到*/
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "xchu_mapping/FloorCoeffs.h"
#include "xchu_mapping/common.h"

class CloudFilter {
 public:
  CloudFilter();

  /**
   * 主函数入口
   */
  void Run();

 private:
  ros::NodeHandle nh_;
  ros::Publisher points_pub_, final_ground_pub_, non_points_pub_, floor_pub_, normal_ground_pub_;
  ros::Subscriber lidar_sub_;

  std::string cloud_topic_;

  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterKeyFrames, downSizeGroundFrames, downSizeNoGroundFrames;
  std::mutex mutex_lock_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue;

  // plane params
  double tilt_deg_ = 0.0;
  double sensor_height_ = 1.75;
  double height_clip_range_ = 2.5;  //[sensor_height - height_clip_range, sensor_height + height_clip_range]
  int floor_pts_thresh_ = 512;
  double floor_normal_thresh_ = 10.0;
  bool use_normal_filtering_ = true;
  double normal_filter_thresh_ = 20.0;
  bool use_outlier_removal_method_ = true;

  /**
   * 点云 callback
   * @param msg
   */
  void PcCB(const sensor_msgs::PointCloud2ConstPtr &msg);

  /**
   * 地面提取：通过法向量过滤以及RANSANC拟合平面
   * @param cloud
   * @param current_header
   * @return
   */
  Eigen::Vector4f DetectPlane(const pcl::PointCloud<PointT>::Ptr &cloud, std_msgs::Header &current_header);

  /**
   * 利用plane cliper去除点云
   * @param src_cloud
   * @param plane
   * @param negative
   * @return
   */
  pcl::PointCloud<PointT>::Ptr PlaneClip(const pcl::PointCloud<PointT>::Ptr &src_cloud,
                                         const Eigen::Vector4f &plane,
                                         bool negative);

  /**
   * 法向量过滤
   * @param cloud
   * @return
   */
  pcl::PointCloud<PointT>::Ptr NormalFiltering(const pcl::PointCloud<PointT>::Ptr &cloud);

};

#endif //SRC_XCHU_MAPPING_SRC_POINTS_FILTER_NODE_H_
