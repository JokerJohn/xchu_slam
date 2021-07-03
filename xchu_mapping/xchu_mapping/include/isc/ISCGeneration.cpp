//
// Created by xchu on 2021/5/17.
//

#include "isc/ISCGeneration.h"

//#define INTEGER_INTENSITY
ISCGeneration::ISCGeneration() {

}

void ISCGeneration::init_param(int rings_in, int sectors_in, double max_dis_in) {
  rings = rings_in;
  sectors = sectors_in;
  max_dis = max_dis_in;
  ring_step = max_dis / rings;
  sector_step = 2 * M_PI / sectors;
  print_param();
  init_color();

  current_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}

void ISCGeneration::init_color(void) {
  for (int i = 0; i < 1; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(0, i * 16, 255));
  }
  for (int i = 0; i < 15; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(0, i * 16, 255));
  }
  for (int i = 0; i < 16; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(0, 255, 255 - i * 16));
  }
  for (int i = 0; i < 32; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(i * 32, 255, 0));
  }
  for (int i = 0; i < 16; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(255, 255 - i * 16, 0));
  }
  for (int i = 0; i < 64; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(i * 4, 255, 0));
  }
  for (int i = 0; i < 64; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(255, 255 - i * 4, 0));
  }
  for (int i = 0; i < 64; i++) {//RGB format
    color_projection.push_back(cv::Vec3b(255, i * 4, i * 4));
  }
}

void ISCGeneration::print_param() {
  std::cout << "The ISC parameters are:" << rings << std::endl;
  std::cout << "number of rings:\t" << rings << std::endl;
  std::cout << "number of sectors:\t" << sectors << std::endl;
  std::cout << "maximum distance:\t" << max_dis << std::endl;
}

ISCDescriptor ISCGeneration::calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud) {

  // sectors和rings是初始化参数，参考scan context
  // 初始化一个sectors*ring的图像
  ISCDescriptor isc = cv::Mat::zeros(cv::Size(sectors, rings), CV_8U);

  // 计算每一个遍历点的ring_id和sector_id
  for (int i = 0; i < (int) filtered_pointcloud->points.size(); i++) {
    ROS_WARN_ONCE(
        "intensity is %f, if intensity showed here is integer format between 1-255, please uncomment #define INTEGER_INTENSITY in iscGenerationClass.cpp and recompile",
        (double) filtered_pointcloud->points[i].intensity);
    double distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x
                                    + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
    if (distance >= max_dis)
      continue;
    double angle = M_PI + std::atan2(filtered_pointcloud->points[i].y, filtered_pointcloud->points[i].x);
    int ring_id = std::floor(distance / ring_step);
    int sector_id = std::floor(angle / sector_step);
    if (ring_id >= rings)
      continue;
    if (sector_id >= sectors)
      continue;
#ifndef INTEGER_INTENSITY
    int intensity_temp = (int) (255 * filtered_pointcloud->points[i].intensity); // 强度是整数
#else
    int intensity_temp = (int) (filtered_pointcloud->points[i].intensity);
#endif
    if (isc.at<unsigned char>(ring_id, sector_id) < intensity_temp)
      isc.at<unsigned char>(ring_id, sector_id) = intensity_temp;  // 像素值存强度？
  }

  return isc;
}

ISCDescriptor ISCGeneration::getLastISCMONO(void) {
  return isc_arr.back();
}

ISCDescriptor ISCGeneration::getLastISCRGB(void) {
  //ISCDescriptor isc = isc_arr.back();
  ISCDescriptor isc_color = cv::Mat::zeros(cv::Size(sectors, rings), CV_8UC3);
  for (int i = 0; i < isc_arr.back().rows; i++) {
    for (int j = 0; j < isc_arr.back().cols; j++) {
      isc_color.at<cv::Vec3b>(i, j) = color_projection[isc_arr.back().at<unsigned char>(i, j)];
    }
  }
  return isc_color;
}

void ISCGeneration::loopDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr &current_pc,
                                  Eigen::Isometry3d &odom) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  ground_filter(current_pc, pc_filtered); // 对z高度进行截取，粗略的去除地面
  ISCDescriptor desc = calculate_isc(pc_filtered); // 计算非地面点云的sc特征
  Eigen::Vector3d current_t = odom.translation(); // 当前odom位置
  //dont change push_back sequence
  if (travel_distance_arr.size() == 0) {
    travel_distance_arr.push_back(0);
  } else {
    // 这里是计算里程距离？
    double dis_temp = travel_distance_arr.back() + std::sqrt((pos_arr.back() - current_t).array().square().sum());
    travel_distance_arr.push_back(dis_temp);
  }
  pos_arr.push_back(current_t); // odom装到这里
  isc_arr.push_back(desc); // isc特征全部装到这里面

  current_frame_id = pos_arr.size() - 1;
  matched_frame_id.clear();

  //search for the near neibourgh pos
  int best_matched_id = 0;
  double best_score = 0.0;
  for (int i = 0; i < (int) pos_arr.size(); i++) { // 遍历之前点云帧的isc特征
    double delta_travel_distance = travel_distance_arr.back() - travel_distance_arr[i]; // 先检测是否存在距离最近的帧
    double pos_distance = std::sqrt((pos_arr[i] - pos_arr.back()).array().square().sum());  // 两帧空间距离
    // 我们认为里程距离越大，空间距离越小，越可能是回环
    if (delta_travel_distance > SKIP_NEIBOUR_DISTANCE && pos_distance < delta_travel_distance * INFLATION_COVARIANCE) {
      double geo_score = 0;
      double inten_score = 0;
      if (is_loop_pair(desc, isc_arr[i], geo_score, inten_score)) { // 计算isc的几何以及强度得分
        if (geo_score + inten_score > best_score) {  // 如果得分满足阈值，则认为是回环位置
          best_score = geo_score + inten_score;
          best_matched_id = i;
        }
      }

    }
  }
  if (best_matched_id != 0) {
    matched_frame_id.push_back(best_matched_id);
    ROS_INFO("received loop closure candidate: current: %d, history %d, total_score %f",
             current_frame_id,
             best_matched_id,
             best_score);
  }

}

std::pair<int, float> ISCGeneration::detectLoopClosureID() {

  int curr_node_idx = pos_arr.size() - 1;

  //search for the near neibourgh pos
  int prev_node_idx = 0;
  double best_score = 0.0;
  for (int i = 0; i < pos_arr.size(); i++) { // 遍历之前点云帧的isc特征
    double delta_travel_distance = travel_distance_arr.back() - travel_distance_arr[i]; // 先检测是否存在距离最近的帧
    double pos_distance = std::sqrt((pos_arr[i] - pos_arr.back()).array().square().sum());  // 两帧空间距离
    // 我们认为里程距离越大，空间距离越小，越可能是回环
    if (delta_travel_distance > SKIP_NEIBOUR_DISTANCE
        && pos_distance < delta_travel_distance * INFLATION_COVARIANCE) {
      double geo_score = 0;
      double inten_score = 0;
      if (is_loop_pair(isc_arr[curr_node_idx],
                       isc_arr[i],
                       geo_score,
                       inten_score)) { // 计算isc的几何以及强度得分
        if (geo_score + inten_score > best_score) {  // 如果得分满足阈值，则认为是回环位置
          best_score = geo_score + inten_score;
          prev_node_idx = i;
        }
      }
    }
  }
  std::pair<int, float> result{-1, 0.0};
  if (prev_node_idx != 0) {
    ROS_WARN("received loop closure candidate: current: %d, history %d, total_score %f",
             curr_node_idx,
             prev_node_idx,
             best_score);
      result = {prev_node_idx, curr_node_idx};
  }

  return result;
}

void ISCGeneration::makeAndSavedec(const pcl::PointCloud<pcl::PointXYZI>::Ptr &current_pc,
                                   pcl::PointXYZI &pose_curr) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  ground_filter(current_pc, pc_filtered); // 对z高度进行截取，粗略的去除地面
  ISCDescriptor desc = calculate_isc(pc_filtered); // 计算非地面点云的sc特征
  Eigen::Vector3d current_t(pose_curr.x, pose_curr.y, pose_curr.z); // 当前odom位置

  //dont change push_back sequence
  if (travel_distance_arr.size() == 0) {
    travel_distance_arr.push_back(0);
  } else {
    // 这里是计算里程距离？
    double dis_temp = travel_distance_arr.back()
        + std::sqrt((pos_arr.back() - current_t).array().square().sum());
    travel_distance_arr.push_back(dis_temp);
  }
  pos_arr.push_back(current_t); // odom装到这里
  isc_arr.push_back(desc); // isc特征全部装到这里面
}

bool ISCGeneration::is_loop_pair(ISCDescriptor &desc1,
                                 ISCDescriptor &desc2,
                                 double &geo_score,
                                 double &inten_score) {
  int angle = 0;
  geo_score = calculate_geometry_dis(desc1, desc2, angle); // 计算几何距离
  if (geo_score > GEOMETRY_THRESHOLD) {
    inten_score = calculate_intensity_dis(desc1, desc2, angle); // 计算强度得分
    if (inten_score > INTENSITY_THRESHOLD) {
      return true;
    }
  }
  return false;
}

double ISCGeneration::calculate_geometry_dis(const ISCDescriptor &desc1, const ISCDescriptor &desc2, int &angle) {
  double similarity = 0.0;

  // 几何结构匹配
  for (int i = 0; i < sectors; i++) { // 列
    int match_count = 0;
    // 矩阵逐元素异或
    for (int p = 0; p < sectors; p++) {
      int new_col = p + i >= sectors ? p + i - sectors : p + i;
      for (int q = 0; q < rings; q++) {  // 行
        // 由于二值矩阵的列向量表示方位角，因此LiDAR的旋转可以通过矩阵的列移动反映．
        // 因此，为了检测视角变化，我们需要计算具有最大几何相似度的列移动
        if ((desc1.at<unsigned char>(q, p) == true && desc2.at<unsigned char>(q, new_col) == true)
            || (desc1.at<unsigned char>(q, p) == false && desc2.at<unsigned char>(q, new_col) == false)) {
          match_count++;
        }

      }
    }
    if (match_count > similarity) {
      similarity = match_count;
      angle = i;
    }

  }
  return similarity / (sectors * rings);
}

double ISCGeneration::calculate_intensity_dis(const ISCDescriptor &desc1, const ISCDescriptor &desc2, int &angle) {
  double difference = 1.0;
  // 密度结构匹配 计算上一步最佳列旋转后的和候选帧的ISC的密度相似度
  // 取ISC对应列的余弦距离的均值
  double angle_temp = angle;
  for (int i = angle_temp - 10; i < angle_temp + 10; i++) {
    int match_count = 0;
    int total_points = 0;
    for (int p = 0; p < sectors; p++) {
      int new_col = p + i;
      if (new_col >= sectors)
        new_col = new_col - sectors;
      if (new_col < 0)
        new_col = new_col + sectors;
      for (int q = 0; q < rings; q++) {
        match_count += abs(desc1.at<unsigned char>(q, p) - desc2.at<unsigned char>(q, new_col));
        total_points++;
      }
    }
    double diff_temp = ((double) match_count) / (sectors * rings * 255);
    if (diff_temp < difference)
      difference = diff_temp;
  }
  return 1 - difference;
}
void ISCGeneration::ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out) {
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(pc_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.9, 30.0);
  pass.filter(*pc_out);
}