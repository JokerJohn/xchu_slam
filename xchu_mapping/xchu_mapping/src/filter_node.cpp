/**
 * @file filter_node.cpp
 * @author XCHU (2022087641@qq.com)
 * @brief  此部分主要做点云过滤的地面分割，其中地面分割采用基本的法向量过滤和ransanc拟合，ax+by+cz+d=0,
           地面参数(a, b, c, d)直接通过msg发布出来，方便后续使用。
           代码比较简单，直接copy的hdl graph slam
 * @version 1.0
 * @date 2020-09-20
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "xchu_mapping/filter_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_node");
  ROS_INFO("\033[1;32m---->\033[0m XCHU Filter Node Started.");

  CloudFilter filter;
  std::thread filter_thread(&CloudFilter::Run, &filter);

  ros::spin();
  return 0;
}

CloudFilter::CloudFilter() : nh_("~") {
  nh_.param<std::string>("cloud_topic", cloud_topic_, "/kitti/velo/pointcloud");
  nh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "velo_link");
  nh_.param<double>("sensor_height", sensor_height_, 1.75);

  downSizeFilterKeyFrames.setLeafSize(filter_size, filter_size, filter_size); // 发布全局地图的采样size,设置小了耗费时间
  downSizeGroundFrames.setLeafSize(0.8, 0.8, 0.8); //  地面的采样点size可以大一些
  downSizeNoGroundFrames.setLeafSize(0.2, 0.2, 0.2);

  // 注意lidar_sub_需要为全局变量，否则callback中不会有数据
  lidar_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(cloud_topic_, 10, &CloudFilter::PcCB, this);
  points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
  final_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_points", 32);
  non_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/no_ground_points", 32);
  normal_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/normal_ground_points", 32);
  floor_pub_ = nh_.advertise<xchu_mapping::FloorCoeffs>("/ground_coeffs", 32);
}

void CloudFilter::Run() {
  while (1) {
    while (!cloud_queue.empty()) {
      //read data 取数据
      mutex_lock_.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*cloud_queue.front(), *pointcloud_in);
      ros::Time pointcloud_time = (cloud_queue.front())->header.stamp;
      std_msgs::Header cloud_header = (cloud_queue.front())->header;
      cloud_queue.pop();
      mutex_lock_.unlock();

      if (pointcloud_in->empty()) {
        ROS_ERROR("Filer Node: cloud is empty !!!");
        continue;
      }
      //remove NAN，针对速腾雷达
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*pointcloud_in, *pointcloud_in, indices);
      // 去除远近的点云
      double r;
      pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      for (auto point:pointcloud_in->points) {
        r = std::sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        if (0.5 < r && r < 60) {
          scan_ptr->points.push_back(point);
        }
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      downSizeFilterKeyFrames.setInputCloud(scan_ptr);
      downSizeFilterKeyFrames.filter(*filtered_scan_ptr);

      //  根据分布或者临近距离去除离群点
      pcl::PointCloud<pcl::PointXYZI>::Ptr sor_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      if (use_outlier_removal_method_) {
        int mean_k = 30;
        double stddev_mul_thresh = 1.0;

        pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
        sor->setMeanK(mean_k);
        sor->setStddevMulThresh(stddev_mul_thresh);
        sor->setInputCloud(filtered_scan_ptr);
        sor->filter(*sor_scan_ptr);
      } else {
        double radius = 0.8;
        int min_neighbors = 5;

        pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
        rad->setRadiusSearch(radius);
        rad->setMinNeighborsInRadius(min_neighbors);
        rad->setInputCloud(filtered_scan_ptr);
        rad->filter(*sor_scan_ptr);
      }

      ros::Time test_time_1 = ros::Time::now();
      // 提取地面
      Eigen::Vector4f floor = DetectPlane(sor_scan_ptr, cloud_header);
      ros::Time test_time_2 = ros::Time::now();
      //    std::cout << "detect plane: " << (test_time_2 - test_time_1) * 1000 << "ms"  << std::endl;

      // publish the detected floor coefficients
      xchu_mapping::FloorCoeffs coeffs;
      coeffs.header.stamp = pointcloud_time;
      coeffs.header.frame_id = lidar_frame_id_;
      if (floor != Eigen::Vector4f::Identity()) {
        coeffs.coeffs.resize(4);
        for (int i = 0; i < 4; i++) {
          coeffs.coeffs[i] = floor[i];
        }
      }
      floor_pub_.publish(coeffs);

      // 实时的点云也发布
      if (points_pub_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2::Ptr pointcloud_current_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*sor_scan_ptr, *pointcloud_current_ptr);
        pointcloud_current_ptr->header.stamp = pointcloud_time;
        pointcloud_current_ptr->header.frame_id = lidar_frame_id_;
        points_pub_.publish(*pointcloud_current_ptr);
      }
    }

    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void CloudFilter::PcCB(const sensor_msgs::PointCloud2ConstPtr &msg) {
  mutex_lock_.lock();
  cloud_queue.push(msg);
  mutex_lock_.unlock();
}

pcl::PointCloud<PointT>::Ptr CloudFilter::PlaneClip(const pcl::PointCloud<PointT>::Ptr &src_cloud,
                                                    const Eigen::Vector4f &plane,
                                                    bool negative) {
  // PlaneClipper3D用任意平面分割点云空间，plane为平面参数
  pcl::PlaneClipper3D<PointT> clipper(plane);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  clipper.clipPointCloud3D(*src_cloud, indices->indices);

  pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*dst_cloud);

  return dst_cloud;
}

pcl::PointCloud<PointT>::Ptr CloudFilter::NormalFiltering(const pcl::PointCloud<PointT>::Ptr &cloud) {
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // 设置10个紧邻点
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(10);
  ne.setViewPoint(0.0f, 0.0f, sensor_height_);
  ne.compute(*normals);

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
  filtered->reserve(cloud->size());

  // 遍历所有的点，法向量满足要求时这个点才被选取
  for (int i = 0; i < cloud->size(); i++) {
    float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
    if (std::abs(dot) > std::cos(normal_filter_thresh_ * M_PI / 180.0)) {
      filtered->push_back(cloud->at(i));
    }
  }

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

Eigen::Vector4f CloudFilter::DetectPlane(const pcl::PointCloud<PointT>::Ptr &cloud, std_msgs::Header &current_header) {

  // compensate the tilt rotation
  Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
  tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg_ * M_PI / 180.0f,
                                                      Eigen::Vector3f::UnitY()).toRotationMatrix();

  // filtering before RANSAC (height and normal filtering)
  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
  filtered = PlaneClip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height_ + height_clip_range_),
                       false);
  filtered = PlaneClip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height_ - height_clip_range_), true);

  // 去掉法向量异常的点, 地面点云的法向量应该朝上
  if (use_normal_filtering_) {
    filtered = NormalFiltering(filtered);
  }
  pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

  pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
  pcl::ExtractIndices<PointT> first_extract;
  first_extract.setInputCloud(filtered);
  first_extract.setIndices(ground_inliers);

  // 简单的法向量过滤后的地面店
  if (normal_ground_pub_.getNumSubscribers()) {
    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

    // normal_ground这里还包括地面之上 法向量垂直的点
    downSizeNoGroundFrames.setInputCloud(filtered);
    downSizeNoGroundFrames.filter(*output_cloud);

    sensor_msgs::PointCloud2::Ptr temp_cloud_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*output_cloud, *temp_cloud_ptr);
    temp_cloud_ptr->header.frame_id = lidar_frame_id_;
    temp_cloud_ptr->header.stamp = current_header.stamp;
    normal_ground_pub_.publish(temp_cloud_ptr);
  }

  // too few points for RANSAC
  if (filtered->size() < floor_pts_thresh_) {
    return Eigen::Vector4f::Identity();
  }

  // RANSAC
  pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
  pcl::RandomSampleConsensus<PointT> ransac(model_p);
  ransac.setDistanceThreshold(0.1);
  ransac.computeModel();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  ransac.getInliers(inliers->indices);

  // too few inliers
  if (inliers->indices.size() < floor_pts_thresh_) {
    return Eigen::Vector4f::Identity();
  }

  // verticality check of the detected floor's normal
  Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);
  // 法向量不合理，最小90+-20度
  double dot = coeffs.head<3>().dot(reference.head<3>());
  if (std::abs(dot) < std::cos(floor_normal_thresh_ * M_PI / 180.0)) {
    // the normal is not vertical
    return Eigen::Vector4f::Identity();
  }

  // make the normal upward
  if (coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
    coeffs *= -1.0f;
  }

  if (final_ground_pub_.getNumSubscribers()) {
    pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*filtered, inliers->indices, *inlier_cloud);

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
    downSizeNoGroundFrames.setInputCloud(inlier_cloud);
    downSizeNoGroundFrames.filter(*output_cloud);

    sensor_msgs::PointCloud2::Ptr temp_cloud_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*output_cloud, *temp_cloud_ptr);
    temp_cloud_ptr->header.stamp = current_header.stamp;
    temp_cloud_ptr->header.frame_id = lidar_frame_id_;
    temp_cloud_ptr->header.stamp = current_header.stamp;
    final_ground_pub_.publish(temp_cloud_ptr);
  }

  // 整个的非地面点
  if (non_points_pub_.getNumSubscribers()) {
    pcl::PointCloud<PointT>::Ptr outlier_cloud(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // 提取非地面点
    extract.filter(*outlier_cloud);

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
    downSizeNoGroundFrames.setInputCloud(outlier_cloud);
    downSizeNoGroundFrames.filter(*output_cloud);

    sensor_msgs::PointCloud2::Ptr temp_cloud_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*output_cloud, *temp_cloud_ptr);
    temp_cloud_ptr->header.stamp = current_header.stamp;
    temp_cloud_ptr->header.frame_id = lidar_frame_id_;
    non_points_pub_.publish(temp_cloud_ptr);
  }

  return Eigen::Vector4f(coeffs);
}


