//
// Created by xchu on 2021/5/12.
//

#ifndef SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_PGO_H_
#define SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_PGO_H_
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
//#include <stat.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

#include <eigen3/Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/dataset.h> //引入头文件

#include "scancontext/tic_toc.h"
#include "scancontext/Scancontext.h"
#include "xchu_mapping/common.h"
#include "isc/ISCGeneration.h"
#include "gps_tools/gpsTools.h"

using namespace gtsam;
using std::cout;
using std::endl;

class PGO {
 public:
  PGO();

  /**
   * 读取gps odom相关信息并加入到因子图中
   */
  void Run();

  /**
   * 回环检测线程
   */
  void LoopClosure();

  void ICPRefine();

  void MapVisualization();

  /**
   * ros service保存地图和实验数据
   * @param req
   * @param res
   * @return
   */
  bool SaveMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * 关闭终端保存地图
   */
  void SaveMap();

 private:
  ros::NodeHandle nh;
  double keyframeMeterGap;
  double movementAccumulation = 1000000.0; // large value means must add the first given frame.
  bool isNowKeyFrame = false;

  int loop_method = 2; // 0:不适用sc, 1:使用sc, 2:使用isc

  std::queue<nav_msgs::Odometry::ConstPtr> odom_queue_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_queue_;
  std::queue<nav_msgs::OdometryConstPtr> gps_queue_;
  std::queue<std::pair<int, int> > loop_queue_;

  std::mutex mutex_;
  std::mutex mKF;
  std::mutex mutex_pg_;  // when pose graph add node
  std::mutex mutex_pose_;

  std::string save_dir_;
  std::string odom_topic_, cloud_topic_, gps_topic_;
  ros::ServiceServer srv_save_map_;


  double time_stamp_ = 0.0;
  bool init_time = false;
  double curr_odom_time_ = 0;
  pcl::PointCloud<PointT>::Ptr curr_frame_;
  pcl::PointCloud<PointT>::Ptr laserCloudMapAfterPGO;

  std::vector<pcl::PointCloud<PointT>::Ptr> keyframeLaserClouds;
  std::vector<std::pair<int, int> > loop_pairs_;
  std::vector<Pose6D> originPoses;
  std::vector<nav_msgs::Odometry::ConstPtr> originOdom;
  std::vector<Pose6D> keyframePoses;
  std::vector<Pose6D> keyframePosesUpdated;
  std::vector<double> keyframeTimes;
  pcl::PointCloud<PointT>::Ptr keyposes_cloud_;
  pcl::PointCloud<PointT>::Ptr keyframePoints;
  pcl::PointCloud<PointT>::Ptr keyframeGpsPoints;

  gtsam::NonlinearFactorGraph gtSAMgraph;
  bool gtSAMgraphMade = false;
  gtsam::Values initialEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values isamCurrentEstimate;

  Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init
  Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero

  noiseModel::Diagonal::shared_ptr priorNoise;
  noiseModel::Diagonal::shared_ptr odomNoise;
  noiseModel::Base::shared_ptr robustLoopNoise;
  noiseModel::Base::shared_ptr robustGPSNoise;

  pcl::KdTreeFLANN<PointT>::Ptr kdtreeHistoryKeyPoses;
  pcl::PointCloud<PointT>::Ptr laserCloudMapPGO;
  pcl::VoxelGrid<PointT> downSizeFilterMapPGO, downSizeFilterICP, downSizePublishCloud;
  pcl::VoxelGrid<PointT> downSizeFilterScancontext;

  // sc loop detection
  SCManager scManager;
  double scDistThres;
  // isc loop detection
  ISCGeneration iscGeneration;

  bool laserCloudMapPGORedraw = true;
  float pose_cov_thre = 0.1;
  float gps_cov_thre = 0.3;
  Eigen::MatrixXd pose_covariance_curr;
  bool useGPS = true;
  nav_msgs::Odometry::ConstPtr curr_gps_;
  bool hasGPSforThisKF = false;
  bool useGpsElevation = true;
  bool gpsOffsetInitialized = false;
  bool orientation_ready_ = false;
  bool init_yaw = false;
  Eigen::Matrix4d gnss_trans;
  Eigen::Vector3d gps_pos_pre;
  Pose6D gps_pose;
  double yaw = 0.0;
  gpsTools gps_tools_;
  double origin_altitude = 0.0, origin_latitude = 0.0, origin_longitude = 0.0;
  double recentOptimizedX = 0.0;
  double recentOptimizedY = 0.0;
  geometry_msgs::Quaternion yaw_quat_;

  ros::Publisher map_pub, odom_pub, final_odom_pub, pose_pub, markers_pub, isc_pub, gps_pose_pub;
  ros::Subscriber points_sub, odom_sub, gps_sub;

  void OdomCB(const nav_msgs::Odometry::ConstPtr &_laserOdometry);

  void PcCB(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes);

  void GpsCB(const nav_msgs::OdometryConstPtr &_gps);

  void InitParams();

  void PublishPoseAndFrame();

  /**
   * 检测潜在的回环帧，装到loop_queue中，在ICPRefine中进一步的确认
   */
  void PerformSCLoopClosure();

  /**
   * 因子图执行优化，并更新key pose
   */
  void ISAM2Update();

  /**
   * 寻找当前位置附近的关键帧点云，并拼接成localmap
   * @param nearKeyframes
   * @param key
   * @param submap_size
   * @param root_idx
   */
  void LoopFindNearKeyframesCloud(pcl::PointCloud<PointT>::Ptr &nearKeyframes,
                                  const int &key,
                                  const int &submap_size,
                                  const int &root_idx);

  inline gtsam::Pose3 Pose6D2Pose3(const Pose6D &p) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
  }

  pcl::PointCloud<PointT>::Ptr TransformCloud2Map(const pcl::PointCloud<PointT>::Ptr &cloudIn, const Pose6D &tf);

  pcl::PointCloud<PointT>::Ptr TransformCloud2Map(pcl::PointCloud<PointT>::Ptr cloudIn, gtsam::Pose3 transformIn);


  /**
   * 位姿图可视化
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray CreateMarker(const ros::Time &stamp);

};

#endif //SRC_XCHU_MAPPING_INCLUDE_XCHU_MAPPING_PGO_H_
