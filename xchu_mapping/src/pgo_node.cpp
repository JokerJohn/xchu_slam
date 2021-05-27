//
// Created by xchu on 2021/5/12.
// 这里的代码框架主要参考SC LEGO LOAM
//
#include "xchu_mapping/pgo.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pgo_node");
  ROS_INFO("\033[1;32m---->\033[0m XCHU PGO Node Started.");

  PGO pgo;
  std::thread loop_detection(&PGO::LoopClosure, &pgo); // 回环检测 2hz
  std::thread icp_calculation(&PGO::ICPRefine, &pgo); // 优化 1hz
  std::thread visualize_map(&PGO::MapVisualization, &pgo);
  std::thread mapping(&PGO::Run, &pgo);

  /*ros::Rate rate(200);
  while (ros::ok()) {
    pgo.Run();
    ros::spinOnce();
    rate.sleep();
  }
  loop_detection.join();
  icp_calculation.join();
  visualize_map.join();*/

  ros::spin();
  return 0;
}

PGO::PGO() : nh("~") {
  InitParams();

  // 换里程计的时候在这里换掉odom和point cloud的topic即可
  points_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic_.c_str(), 100, &PGO::PcCB, this);
  odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_.c_str(), 100, &PGO::OdomCB, this);
  // gps暂不可用
  if (useGPS) {
    gps_sub = nh.subscribe<nav_msgs::Odometry>("/gps_odom", 100, &PGO::GpsCB, this);
  }
  srv_save_map_ = nh.advertiseService("/save_map", &PGO::SaveMap, this);

  final_odom_pub = nh.advertise<nav_msgs::Odometry>("/laser_odom_after_pgo", 100);
  gps_pose_pub = nh.advertise<sensor_msgs::PointCloud2>("/gps_poses", 10);
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 100);
  pose_pub = nh.advertise<sensor_msgs::PointCloud2>("/key_poses", 10);  // key pose
  markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers", 16);
  if (loop_method == 2)
    isc_pub = nh.advertise<sensor_msgs::Image>("/isc", 100);
}

void PGO::InitParams() {
  nh.param<std::string>("save_dir_", save_dir_, "/home/xchu/workspace/xchujwu_slam/src/xchu_mapping/pcd/");
  nh.param<std::string>("odom_topic", odom_topic_, "/laser_odom_to_init");
  nh.param<std::string>("cloud_topic", cloud_topic_, "/filtered_points");
  nh.param<std::string>("gps_topic", gps_topic_, "/kitti/oxts/gps/fix");
  nh.param<int>("loop_method", loop_method, 1);
  nh.param<bool>("use_gps", useGPS, false);
  nh.param<bool>("use_kitti", use_kitti_, false);
  nh.param<std::string>("lidar_frame_id", lidar_frame_id_, "velo_link");
  nh.param<std::string>("world_frame_id", world_frame_id_, "map");

  if (use_kitti_) {
    velo2camera.setIdentity();
    velo2camera << 0, -1, 0, 0,
        0, 0, -1, 0,
        1, 0, 0, -0.08,
        0, 0, 0, 1;
  }

  //  std::cout << "topic: " << odom_topic_ << "," << cloud_topic_ << std::endl;
  // 每隔2m选取关键帧
  keyframeMeterGap = 1.5;

  // scan contetx的阈值和参数配置
  scDistThres = 0.2;
  scManager.setSCdistThres(scDistThres);

  // intensity scan context参数配置
  int sector_width = 60;
  int ring_height = 60;
  double max_dis = 40.0;
  iscGeneration.init_param(ring_height, sector_width, max_dis);

  curr_frame_.reset(new pcl::PointCloud<PointT>());
  laserCloudMapAfterPGO.reset(new pcl::PointCloud<PointT>());
  laserCloudMapPGO.reset(new pcl::PointCloud<PointT>());
  keyposes_cloud_.reset(new pcl::PointCloud<PointT>());
  keyframePoints.reset(new pcl::PointCloud<PointT>());
  keyframeGpsPoints.reset(new pcl::PointCloud<PointT>());
  kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointT>());

  downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
  downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);
  downSizePublishCloud.setLeafSize(filter_size, filter_size, filter_size);
  downSizeFilterMapPGO.setLeafSize(filter_size, filter_size, filter_size);

  // gtsam params
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);

  // priorNoise, 这里里程计和先验噪声一般非常小，表明可靠
  gtsam::Vector priorNoiseVector6(6);
  priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
  priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

  // odom factor noise
  gtsam::Vector odomNoiseVector6(6);
  odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

  // loop factor noise
  double loopNoiseScore = 0.3;
  gtsam::Vector robustNoiseVector6(6);
  robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
  robustLoopNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
      gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

  // gps factor noise
  double bigNoiseTolerentToXY = 1000000000.0; // 1e9
  double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop closing, use this value bigger
  gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
  robustNoiseVector3
      << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
  robustGPSNoise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
      gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));
}

void PGO::OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
  mutex_.lock();
  odom_queue_.push(msg);
  mutex_.unlock();

  // originOdom主要存储里程计的原始pose,避免到odom_node中记录实验结果，更好的解耦。
  originOdom.push_back(msg);
}

void PGO::PcCB(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes) {
  mutex_.lock();
  cloud_queue_.push(_laserCloudFullRes);
  mutex_.unlock();
}

void PGO::GpsCB(const nav_msgs::OdometryConstPtr &_gps) {
  if (useGPS) {
    mutex_.lock();
    gps_queue_.push(_gps);
    mutex_.unlock();
  }
}

void PGO::Run() {
  while (1) {
    while (!cloud_queue_.empty() && !odom_queue_.empty()) {
      mutex_.lock();
      double time2 = odom_queue_.front()->header.stamp.toSec();
      double time3 = cloud_queue_.front()->header.stamp.toSec();
      // align timestamp
      while (!odom_queue_.empty() && (time2 < time3 - 0.5 * 0.1)) {
        ROS_WARN("time stamp unaligned error and odometry discarded, pls check your data -->  optimization");
        odom_queue_.pop();
        mutex_.unlock();
        break;
      }
      while (!cloud_queue_.empty() && (time3 < time2 - 0.5 * 0.1)) {
        ROS_WARN("time stamp unaligned error and pointCloud discarded, pls check your data -->  optimization");
        cloud_queue_.pop();
        mutex_.unlock();
        break;
      }

      if (!init_time) {
        // 主要是为了将实验结果转tum格式，需要记录第一帧的时间戳
        time_stamp_ = time3;
        init_time = true;
      }
      curr_odom_time_ = time3;
      curr_frame_->clear();
      pcl::PointCloud<PointT>::Ptr thisKeyFrame(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*cloud_queue_.front(), *thisKeyFrame);
      nav_msgs::OdometryConstPtr odom_curr = odom_queue_.front();
      Pose6D pose_curr = Odom2Pose6D(odom_curr);

      // align gps
      while (!gps_queue_.empty()) {
        auto time4 = gps_queue_.front()->header.stamp.toSec();
        curr_gps_ = gps_queue_.front();
        //ROS_WARN("GNSS DIFF:%f", time4 - curr_odom_time_);
        if (abs(time4 - curr_odom_time_) < 0.2) {
          //ROS_WARN("use gps");
          hasGPSforThisKF = true;
          gps_pose = GeometryToPose6D(*curr_gps_);
        } else {
          hasGPSforThisKF = false;
        }
        gps_queue_.pop();
      }
      cloud_queue_.pop();
      odom_queue_.pop();
      mutex_.unlock();

      odom_pose_curr = pose_curr;

      // 太近的点云帧舍弃，否则位子图太大，速度慢
      double delta_translation = std::sqrt(std::pow((odom_pose_prev.x - odom_pose_curr.x), 2)
                                               + std::pow((odom_pose_prev.y - odom_pose_curr.y), 2)
                                               + std::pow((odom_pose_prev.z - odom_pose_curr.z), 2));
      movementAccumulation += delta_translation;
      if (movementAccumulation > keyframeMeterGap) {
        isNowKeyFrame = true;
        movementAccumulation = 0.0;
      } else {
        isNowKeyFrame = false;
      }
      odom_pose_prev = odom_pose_curr;

      if (!isNowKeyFrame)
        continue;

      // Save data and Add consecutive node
      mKF.lock();
      keyframeLaserClouds.push_back(thisKeyFrame);
      keyframePoses.push_back(pose_curr);
      originPoses.push_back(pose_curr);
      keyframePosesUpdated.push_back(pose_curr);
      keyframeTimes.push_back(curr_odom_time_);

      pcl::PointXYZI point;
      point.x = pose_curr.x;
      point.y = pose_curr.y;
      point.z = pose_curr.z;
      keyframePoints->points.push_back(point);
      point.z = 0;
      // keyposes_cloud_主要是里程计的xy坐标，回环检测搜索临近历史帧的时候距离不考虑z的维度
      keyposes_cloud_->points.push_back(point);
      laserCloudMapPGORedraw = true;
      mKF.unlock();

      if (loop_method == 1) {
        // use sc detector
        scManager.makeAndSaveScancontextAndKeys(*thisKeyFrame);
      } else if (loop_method == 2) {
        // use isc detector
        iscGeneration.makeAndSavedec(thisKeyFrame, point);
        // 发布isc图像
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = world_frame_id_;
        out_msg.header.stamp = ros::Time().fromSec(curr_odom_time_);
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = iscGeneration.getLastISCRGB();
        isc_pub.publish(out_msg.toImageMsg());
      }

      // 更新因子图
      if (!gtSAMgraphMade) {
        const int init_node_idx = 0;
        gtsam::Pose3 poseOrigin = Pose6D2Pose3(keyframePoses.at(init_node_idx));
        mutex_pg_.lock();
        {
          // prior factor
          gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
          initialEstimate.insert(init_node_idx, poseOrigin);
          ISAM2Update();
        }
        mutex_pg_.unlock();
        gtSAMgraphMade = true;
        cout << "posegraph prior node " << init_node_idx << " added" << endl;
      } else {
        const int prev_node_idx = keyframePoses.size() - 2;
        const int curr_node_idx = keyframePoses.size() - 1;

        gtsam::Pose3 poseFrom = Pose6D2Pose3(keyframePoses.at(prev_node_idx));
        gtsam::Pose3 poseTo = Pose6D2Pose3(keyframePoses.at(curr_node_idx));

        mutex_pg_.lock();
        {
          // odom factor
          gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx,
                                                            curr_node_idx,
                                                            poseFrom.between(poseTo),
                                                            odomNoise));
          ROS_WARN("odom factor added at node %d ", curr_node_idx);
          if (hasGPSforThisKF) {
            // 里程计位姿协方差较大时候加入
            //            if (pose_covariance_curr(3, 3) < pose_cov_thre && pose_covariance_curr(4, 4) < pose_cov_thre)
            //              hasGPSforThisKF = false;
            // gps噪声过大不适用
            float noise_x = curr_gps_->pose.covariance[0];
            float noise_y = curr_gps_->pose.covariance[7];
            float noise_z = curr_gps_->pose.covariance[14];
            if (noise_x > gps_cov_thre || noise_y > gps_cov_thre)
              hasGPSforThisKF = false;
            recentOptimizedX = gps_pose.x;
            recentOptimizedY = gps_pose.y;
            double curr_altitude_offseted = gps_pose.z;
            // GPS too noisy, skip
            if (!useGpsElevation) {
              curr_altitude_offseted = pose_curr.z;  // gps的z一般不可信
              noise_z = 0.01;
            }
            // GPS not properly initialized (0,0,0)
            if (abs(recentOptimizedX) < 1e-6 && abs(recentOptimizedY) < 1e-6)
              hasGPSforThisKF = false;
            // 添加GPS因子时候必须先吧经纬高坐标转换到当前世界坐标系下
            if (hasGPSforThisKF) {
              //              std::cout << "pose cov: " << pose_covariance_curr(3, 3) << "," << pose_covariance_curr(4, 4) << ","
              //                        << noise_z << std::endl;
              //              //std::cout << "pose cov: " << pose_covariance_curr << std::endl;
              //              std::cout << "noise: " << noise_x << "," << noise_y << "," << noise_z << std::endl;

              // 保存起来
              PointT gpst;
              gpst.x = gps_pose.x;
              gpst.y = gps_pose.y;
              gpst.z = gps_pose.z;
              keyframeGpsPoints->push_back(gpst);

              gtsam::Vector Vector3(3);
              Vector3 << noise_x, noise_y, noise_z;
              robustGPSNoise = gtsam::noiseModel::Robust::Create(
                  gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                  gtsam::noiseModel::Diagonal::Variances(Vector3));

              mutex_pose_.lock();
              gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY, curr_altitude_offseted);
              gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
              mutex_pose_.unlock();
              ROS_WARN("GPS factor added at node %d ", curr_node_idx);
            }
          }
          initialEstimate.insert(curr_node_idx, poseTo);
          ISAM2Update();
        }
        mutex_pg_.unlock();

        if (curr_node_idx % 100 == 0)
          cout << "posegraph odom node " << curr_node_idx << " added." << endl;
      }
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void PGO::PerformSCLoopClosure() {
  if (keyframePoses.size() < 30)
    return;

  if (loop_method == 0) {
    // 寻找最近的pose
    int curr_node_idx = keyframePosesUpdated.size() - 1;

    pcl::PointXYZI currentRobotPosPoint;
    currentRobotPosPoint.x = keyframePosesUpdated[curr_node_idx].x;
    currentRobotPosPoint.y = keyframePosesUpdated[curr_node_idx].y;
    currentRobotPosPoint.z = 0;

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(keyposes_cloud_);
    kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint,
                                        20.0,
                                        pointSearchIndLoop,
                                        pointSearchSqDisLoop,
                                        0);
    int closestHistoryFrameID = -1;
    for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
      int id = pointSearchIndLoop[i];
      if (std::abs(keyframeTimes[id] - curr_odom_time_) > 30.0) {
        closestHistoryFrameID = id;
        break;
      }
    }
    if (closestHistoryFrameID != -1) {
      int prev_node_idx = closestHistoryFrameID;
      double
          distance =
          sqrt(std::pow((keyframePosesUpdated[prev_node_idx].x - keyframePosesUpdated[curr_node_idx].x), 2) +
              std::pow((keyframePosesUpdated[prev_node_idx].y - keyframePosesUpdated[curr_node_idx].y), 2));
      std::cout << "distance: " << distance << std::endl;

      if (distance < 30.0) {
        mutex_.lock();
        loop_queue_.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        mutex_.unlock();
      } else {
        ROS_ERROR("BAD detection RESULT..");
      }
    }
  } else if (loop_method == 1) {
    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if (SCclosestHistoryFrameID != -1) {
      const int prev_node_idx = SCclosestHistoryFrameID;
      const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
      ROS_WARN("Loop detected! - between %d and %d", prev_node_idx, curr_node_idx);

      double
          distance =
          sqrt(std::pow((keyframePosesUpdated[prev_node_idx].x - keyframePosesUpdated[curr_node_idx].x), 2) +
              std::pow((keyframePosesUpdated[prev_node_idx].y - keyframePosesUpdated[curr_node_idx].y), 2));
      std::cout << "distance: " << distance << std::endl;

      if (distance > 20.0) {
        ROS_ERROR("BAD SC RESULT..");
      } else {
        mutex_.lock();
        loop_queue_.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        mutex_.unlock();
      }
    }
  } else if (loop_method == 2) {
    // 使用ISC
    auto detectResult = iscGeneration.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if (SCclosestHistoryFrameID != -1) {
      const int prev_node_idx = SCclosestHistoryFrameID;
      const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
      ROS_WARN("Loop detected! - between %d and %d", prev_node_idx, curr_node_idx);

      double
          distance =
          sqrt(std::pow((keyframePosesUpdated[prev_node_idx].x - keyframePosesUpdated[curr_node_idx].x), 2) +
              std::pow((keyframePosesUpdated[prev_node_idx].y - keyframePosesUpdated[curr_node_idx].y), 2));
      std::cout << "distance: " << distance << std::endl;

      if (distance > 20.0) {
        ROS_ERROR("BAD ISC RESULT..");
      } else {
        mutex_.lock();
        loop_queue_.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        mutex_.unlock();
      }
    }
  }
}

void PGO::LoopClosure() {
  ros::Rate rate(2.0);
  while (1) {
    rate.sleep();
    PerformSCLoopClosure();

    // 位姿图可视化
    if (markers_pub.getNumSubscribers() && keyframePosesUpdated.size() > 2) {
      visualization_msgs::MarkerArray markers = CreateMarker(ros::Time::now());
      markers_pub.publish(markers);
    }
    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void PGO::ICPRefine() {
  //ros::Rate rate(10.0);
  while (1) {
    while (!loop_queue_.empty()) {
      if (loop_queue_.size() > 30) {
        ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ...");
      }

      mutex_.lock();
      std::pair<int, int> loop_idx_pair = loop_queue_.front();
      loop_queue_.pop();
      mutex_.unlock();

      // 开始icp
      const int _loop_kf_idx = loop_idx_pair.first;
      const int _curr_kf_idx = loop_idx_pair.second;

      // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
      int historyKeyframeSearchNum = 25;
      pcl::PointCloud<PointT>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointT>());
      pcl::PointCloud<PointT>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointT>());
      LoopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx
      LoopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx);

      pcl::IterativeClosestPoint<PointT, PointT> icp;
      icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
      icp.setMaximumIterations(100);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-6);
      icp.setRANSACIterations(0);

      // Align pointclouds
      icp.setInputSource(cureKeyframeCloud);
      icp.setInputTarget(targetKeyframeCloud);
      pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
      icp.align(*unused_result);

      double final_score = icp.getFitnessScore();
      float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe.
      if (icp.hasConverged() == false || final_score > loopFitnessScoreThreshold) {
        ROS_INFO("LOOP REJECT %f", final_score);
        return;
      } else {
        ROS_WARN("LOOP DETECT  SUCCESS %f,", final_score);
      }

      // 闭环成功的存到这里
      loop_pairs_.push_back(std::pair<int, int>(_loop_kf_idx, _curr_kf_idx));

      gtsam::Vector robustNoiseVector6(6);
      robustNoiseVector6 << final_score, final_score, final_score, final_score, final_score, final_score;
      robustLoopNoise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
          gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

      // Get pose transformation
      //      float x, y, z, roll, pitch, yaw;
      //      Eigen::Affine3f correctionLidarFrame;
      //      correctionLidarFrame = icp.getFinalTransformation();
      //      pcl::getTranslationAndEulerAngles(correctionLidarFrame, x, y, z, roll, pitch, yaw);
      // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
      Pose6D t_pose = Matrix2Pose6D(icp.getFinalTransformation().cast<double>());
      gtsam::Pose3 poseFrom = Pose6D2Pose3(t_pose);
      gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

      mutex_pg_.lock();
      gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(_loop_kf_idx,
                                                        _curr_kf_idx,
                                                        poseFrom.between(poseTo),
                                                        robustLoopNoise));
      ISAM2Update();
      mutex_pg_.unlock();
    }

    // wait (must required for running the while loop)
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
  // rate.sleep();
}

void PGO::MapVisualization() {
  ros::Rate rate(0.1);
  while (ros::ok()) {
    rate.sleep();
    if (keyframeLaserClouds.size() > 1 && keyframePosesUpdated.size() > 1) {
      PublishPoseAndFrame();
    }
  }
  // 关闭终端是保存地图
  ROS_WARN("SAVE MAP AND G2O..");
  ISAM2Update(); // 最后一次更新全局地图
  SaveMap();
}

void PGO::ISAM2Update() {
  // called when a variable added
  // 为什么更新2次，清点进去查看api介绍
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  // 边缘化，获取协方差，但这里pose的协方差肯定不准
  isamCurrentEstimate = isam->calculateEstimate();
  pose_covariance_curr = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

  mKF.lock();
  for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size()); node_idx++) {
    pcl::PointXYZI &p2 = keyposes_cloud_->points[node_idx];
    pcl::PointXYZI &p3 = keyframePoints->points[node_idx];
    Pose6D &p = keyframePosesUpdated[node_idx];
    p3.x = p2.x = p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
    p3.y = p2.y = p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
    p3.z = p2.z = p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
    p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
    p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
    p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    p.cov = isam->marginalCovariance(node_idx);
  }
  mKF.unlock();
}

void PGO::LoopFindNearKeyframesCloud(pcl::PointCloud<PointT>::Ptr &nearKeyframes,
                                     const int &key,
                                     const int &submap_size,
                                     const int &root_idx) {
  // extract and stacking near keyframes (in global coord)
  nearKeyframes->clear();
  for (int i = -submap_size; i <= submap_size; ++i) {
    int keyNear = root_idx + i;
    if (keyNear < 0 || keyNear >= keyframeLaserClouds.size())
      continue;

    mKF.lock();
    *nearKeyframes += *TransformCloud2Map(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
    mKF.unlock();
  }

  if (nearKeyframes->empty())
    return;

  // downsample near keyframes
  pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>());
  downSizeFilterICP.setInputCloud(nearKeyframes);
  downSizeFilterICP.filter(*cloud_temp);
  *nearKeyframes = *cloud_temp;
}

pcl::PointCloud<PointT>::Ptr PGO::TransformCloud2Map(const pcl::PointCloud<PointT>::Ptr &cloudIn, const Pose6D &tf) {
  pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());

  int cloudSize = cloudIn->size();
  cloudOut->resize(cloudSize);

  Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

  int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
  for (int i = 0; i < cloudSize; ++i) {
    const auto &pointFrom = cloudIn->points[i];
    cloudOut->points[i].x =
        transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
    cloudOut->points[i].y =
        transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
    cloudOut->points[i].z =
        transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
    cloudOut->points[i].intensity = pointFrom.intensity;
  }

  return cloudOut;
}

bool PGO::SaveMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_WARN("SAVE MAP AND G2O..");
  ISAM2Update(); // 最后一次更新全局地图

  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  std::string file_path = save_dir_ + stamp + "/";
  std::cout << "file path: " << file_path << std::endl;
  if (0 != access(file_path.c_str(), 2)) {
    mkdir(file_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    ROS_INFO("mkdir filepath %s", file_path.c_str());
  }

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
  for (int i = 0; i < keyframeLaserClouds.size(); ++i) {
    *map += *TransformCloud2Map(keyframeLaserClouds[i], keyframePosesUpdated[i]);
  }
  map->width = map->points.size();
  map->height = 1;
  map->is_dense = false;

  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*keyframePoints, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;
  downSizeFilterMapPGO.setInputCloud(map);
  downSizeFilterMapPGO.filter(*map);

  pcl::io::savePCDFile(file_path + "trajectory.pcd", *poses);
  pcl::io::savePCDFile(file_path + "finalMap.pcd", *map);
  std::cout << "save odom csv files and g2o" << std::endl;


  // 生成地图原点
/*  if (useGPS) {
    std::ofstream out;
    out.open(file_path + "map_origin.txt", std::ios::out);
    out << origin_latitude << " " << origin_longitude << " " << origin_altitude;
    out.close();
  }*/

  // 保存odom的csv
  ROS_WARN("save odom csv files and xchu_g2o");
  std::ofstream outFile, outFile2, outFile3;


  // kitti txt
  outFile.open(file_path + "odom_tum.txt", std::ios::out);
  outFile3.open(file_path + "lidar_odom.txt", std::ios::out);

  for (int j = 0; j < keyframePosesUpdated.size(); ++j) {

    //Matrix4d pose2 = Pose6D2Matrix(originPoses[j]);
    Pose6D new_pose = keyframePosesUpdated[j];
    // kitti的gt是相机pose,所以需要把雷达pose转换到相机上。
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (use_kitti_)
      pose = velo2camera * Pose6D2Matrix(new_pose);
    else
      pose = Pose6D2Matrix(new_pose);

    double time = keyframeTimes[j] - time_stamp_;
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);
    outFile << std::setprecision(12) << time << " " << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << " "
            << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
            << std::endl;
  }

  // 写odom.txt
  std::cout << "write lidar odom: " << originOdom.size() << std::endl;
  for (int j = 0; j < originOdom.size(); ++j) {
    nav_msgs::Odometry msg = *originOdom[j];

    Matrix4d pose2 = GeometryToEigen(msg).matrix();
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (use_kitti_)
      pose = velo2camera * pose2;
    else
      pose = pose2;

    double time = msg.header.stamp.toSec() - time_stamp_;
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);
    outFile3 << std::setprecision(12) << time << " " << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << " "
             << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
             << std::endl;
  }
  outFile.close();
  outFile3.close();

  // 同事保存g2o文件
  gtsam::writeG2o(gtSAMgraph, isamCurrentEstimate, file_path + "pose_graph.g2o");
  ROS_WARN("Save map. pose size: %d", poses->points.size());

  return true;
}

void PGO::SaveMap() {
  ROS_WARN("SAVE MAP AND G2O..");
  ISAM2Update(); // 最后一次更新全局地图

  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();

  std::string file_path = save_dir_ + stamp + "/";
  std::cout << "file path: " << file_path << std::endl;
  if (0 != access(file_path.c_str(), 2)) {
    mkdir(file_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    ROS_INFO("mkdir filepath %s", file_path.c_str());
  }

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
  for (int i = 0; i < keyframeLaserClouds.size(); ++i) {
    *map += *TransformCloud2Map(keyframeLaserClouds[i], keyframePosesUpdated[i]);
  }
  map->width = map->points.size();
  map->height = 1;
  map->is_dense = false;

  //  map_no_ground->width = map_no_ground->points.size();
  //  map_no_ground->height = 1;
  //  map_no_ground->is_dense = false;
  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*keyframePoints, *poses);
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;
  downSizeFilterMapPGO.setInputCloud(map);
  downSizeFilterMapPGO.filter(*map);

  pcl::io::savePCDFile(file_path + "trajectory.pcd", *poses);
  pcl::io::savePCDFile(file_path + "finalMap.pcd", *map);

  std::cout << "save odom csv files and g2o" << std::endl;


  // 生成地图原点
  if (useGPS && gpsOffsetInitialized) {
    std::ofstream out;
    out.open(file_path + "map_origin.txt", std::ios::out);
    out << origin_latitude << " " << origin_longitude << " " << origin_altitude;
    out.close();
  }

  // 保存odom的csv
  ROS_WARN("save odom csv files and xchu_g2o");
  std::ofstream outFile, outFile2, outFile3;

  // csv
  /*outFile.open(file_path + "odom_final.csv", std::ios::out);
  outFile2.open(file_path + "odom.csv", std::ios::out);
  outFile << "stamp,x,y,z,roll,pitch,yaw" << std::endl;
  outFile2 << "stamp,x,y,z,roll,pitch,yaw" << std::endl;
  for (int j = 0; j < keyframePosesUpdated.size(); ++j) {
    outFile << keyframeTimes[j] << ","
            << keyframePosesUpdated[j].x << "," << keyframePosesUpdated[j].y << "," << keyframePosesUpdated[j].z << ","
            << keyframePosesUpdated[j].roll << "," << keyframePosesUpdated[j].pitch << ","
            << keyframePosesUpdated[j].yaw
            <<
            endl;

    outFile2 << keyframeTimes[j] << ","
             << originPoses[j].x << "," << originPoses[j].y << "," << originPoses[j].z << ","
             << originPoses[j].roll << "," << originPoses[j].pitch << "," << originPoses[j].yaw
             <<
             endl;
  }*/


  // kitti txt
  outFile.open(file_path + "odom_tum.txt", std::ios::out);
  //outFile2.open(file_path + "odom.txt", std::ios::out);
  outFile3.open(file_path + "lidar_odom.txt", std::ios::out);

  /* Matrix4d velo2camera;
   velo2camera << 0, -1, 0, 0,
       0, 0, -1, 0,
       1, 0, 0, -0.08,
       0, 0, 0, 1;*/

  for (int j = 0; j < keyframePosesUpdated.size(); ++j) {

    //Matrix4d pose2 = Pose6D2Matrix(originPoses[j]);
    Pose6D new_pose = keyframePosesUpdated[j];
    // kitti的gt是相机pose,所以需要把雷达pose转换到相机上。
    Matrix4d pose =/* velo2camera * */Pose6D2Matrix(new_pose);

//    outFile << std::setprecision(10) << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " " << pose(0, 3) << " "
//            << pose(1, 0) << " " << pose(1, 1) << " " << pose(1, 2) << " " << pose(1, 3) << " "
//            << pose(2, 0) << " " << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3)
//            << endl;

//    outFile2 << std::setprecision(10) << pose2(0, 0) << " " << pose2(0, 1) << " " << pose2(0, 2) << " " << pose2(0, 3)
//             << " "
//             << pose2(1, 0) << " " << pose2(1, 1) << " " << pose2(1, 2) << " " << pose2(1, 3) << " "
//             << pose2(2, 0) << " " << pose2(2, 1) << " " << pose2(2, 2) << " " << pose2(2, 3)
//             << endl;

    double time = keyframeTimes[j] - time_stamp_;
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);
    outFile << std::setprecision(12) << time << " " << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << " "
            << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
            << std::endl;
  }

  // 写odom.txt
  std::cout << "write lidar odom: " << originOdom.size() << std::endl;
  for (int j = 0; j < originOdom.size(); ++j) {
    nav_msgs::Odometry msg = *originOdom[j];

    Matrix4d pose2 = GeometryToEigen(msg).matrix();
    Matrix4d pose =  /*velo2camera*/ pose2;

    double time = msg.header.stamp.toSec() - time_stamp_;
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0).matrix();
    Eigen::Quaterniond quat(rot);
    outFile3 << std::setprecision(12) << time << " " << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << " "
             << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
             << std::endl;
  }

  outFile.close();
  //  outFile2.close();
  outFile3.close();

  // 同事保存g2o文件
  gtsam::writeG2o(gtSAMgraph, isamCurrentEstimate, file_path + "pose_graph.g2o");
  ROS_WARN("Save map. pose size: %d", poses->points.size());
}

void PGO::PublishPoseAndFrame() {
  // pub odom and path
  if (keyframePoints->empty() || keyframePosesUpdated.size() < 1)
    return;

  // publish map every 2 frame
  int SKIP_FRAMES = 2;
  int counter = 0;
  laserCloudMapPGO->clear();

  mKF.lock();
  for (int node_idx = 0; node_idx < int(keyframePosesUpdated.size()) - 1;
       node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
  {
    const Pose6D &pose_est = keyframePosesUpdated.at(node_idx); // upodated poses
    if (counter % SKIP_FRAMES == 0) {
      *laserCloudMapPGO += *TransformCloud2Map(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
    }
    counter++;
  }
  mKF.unlock();

  // key poses
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*keyframePoints, msg);
  msg.header.frame_id = world_frame_id_;
  msg.header.stamp = ros::Time::now();
  pose_pub.publish(msg);

  if (useGPS) {
    sensor_msgs::PointCloud2 gps_msg;
    pcl::toROSMsg(*keyframeGpsPoints, gps_msg);
    gps_msg.header.frame_id = world_frame_id_;
    gps_msg.header.stamp = ros::Time::now();
    gps_pose_pub.publish(gps_msg);
  }

  // publish map
  downSizePublishCloud.setInputCloud(laserCloudMapPGO);
  downSizePublishCloud.filter(*laserCloudMapPGO);
  sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
  pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
  laserCloudMapPGOMsg.header.frame_id = world_frame_id_;
  map_pub.publish(laserCloudMapPGOMsg);
}

visualization_msgs::MarkerArray PGO::CreateMarker(const ros::Time &stamp) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker traj_marker, edge_marker, loop_marker;

  // node markers
  traj_marker.header.frame_id = world_frame_id_;
  traj_marker.header.stamp = stamp;
  traj_marker.ns = "nodes";
  traj_marker.id = 0;
  traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  traj_marker.pose.orientation.w = 1.0;
  traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 1.5;
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 1.0;
  traj_marker.color = color;

  // edge markers
  edge_marker.header.frame_id = world_frame_id_;
  edge_marker.header.stamp = stamp;
  edge_marker.ns = "edges";
  edge_marker.id = 1;
  edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
  traj_marker.action = edge_marker.action = visualization_msgs::Marker::ADD;

  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.5;
  edge_marker.color.a = 0.5;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;

  for (int j = 0; j < keyframePosesUpdated.size() - 2; ++j) {
    geometry_msgs::Point pt, pt2;
    pt.x = keyframePosesUpdated[j].x;
    pt.y = keyframePosesUpdated[j].y;
    pt.z = keyframePosesUpdated[j].z;

    pt2.x = keyframePosesUpdated[j + 1].x;
    pt2.y = keyframePosesUpdated[j + 1].y;
    pt2.z = keyframePosesUpdated[j + 1].z;

    traj_marker.points.push_back(pt);
    edge_marker.points.push_back(pt);
    edge_marker.points.push_back(pt2);
    marker_array.markers.push_back(edge_marker);
    marker_array.markers.push_back(traj_marker);
  }


  // loop markers
  if (loop_pairs_.size() > 0) {
    loop_marker.header.frame_id = world_frame_id_;
    loop_marker.header.stamp = stamp;
    loop_marker.ns = "loop";
    loop_marker.id = 2;
    loop_marker.type = visualization_msgs::Marker::LINE_STRIP;

    loop_marker.pose.orientation.w = 1.0;
    loop_marker.scale.x = loop_marker.scale.y = loop_marker.scale.z = 1.0;
    loop_marker.color.r = 1.0;
    loop_marker.color.g = 0.0;
    loop_marker.color.b = 0.0;
    loop_marker.color.a = 0.5;

    for (int j = 0; j < loop_pairs_.size(); ++j) {
      int loop_id = loop_pairs_[j].first;
      int curr_id = loop_pairs_[j].second;

      geometry_msgs::Point pt, pt2;
      pt.x = keyframePosesUpdated[loop_id].x;
      pt.y = keyframePosesUpdated[loop_id].y;
      pt.z = keyframePosesUpdated[loop_id].z;
      pt2.x = keyframePosesUpdated[curr_id].x;
      pt2.y = keyframePosesUpdated[curr_id].y;
      pt2.z = keyframePosesUpdated[curr_id].z;

      loop_marker.points.push_back(pt);
      loop_marker.points.push_back(pt2);
      marker_array.markers.push_back(loop_marker);
    }
  }

  return marker_array;
}
