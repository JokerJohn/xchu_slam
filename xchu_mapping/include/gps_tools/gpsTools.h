//
// Created by echo on 2019/11/26.
//

#ifndef PCD_COMPARE_GPSTOOLS_H
#define PCD_COMPARE_GPSTOOLS_H
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <Eigen/Core>
#define DEG_TO_RAD 0.01745329252
#define EARTH_MAJOR 6378137.0            ///< WGS84 MAJOR AXIS
#define EARTH_MINOR 6356752.31424518    ///< WGS84 MINOR AXIS
class gpsTools {
public:
	gpsTools(){lla_origin_.setIdentity();};
	//1.ros msg 转 eigen
	static Eigen::Vector3d GpsMsg2Eigen(const sensor_msgs::NavSatFix &gps_msgs);
	void updateGPSpose(const sensor_msgs::NavSatFix &gps_msgs);
	//2. LLA经度(longitude),纬度(latitude)和高度(altitude)经纬高坐标系 转(Earth-Centered, Earth-Fixed)
	// Z轴指向指向北，但不完全精确地与地球转动轴重合。转动轴有微小“摆动”，称之为“极运动(polar motion)”
	//X轴在球面上与格林威治线和赤道的交点
	Eigen::Vector3d LLA2ECEF(const Eigen::Vector3d &lla);
	Eigen::Vector3d ECEF2LLA(const Eigen::Vector3d &ecef);
	Eigen::Vector3d ECEF2ENU(const Eigen::Vector3d &ecef);
	Eigen::Vector3d ENU2ECEF(const Eigen::Vector3d &enu);
	//变量部分
	//1.lla的起点
	Eigen::Vector3d lla_origin_;
	//2.enu下的坐标
	Eigen::Vector3d gps_pos_;
private:
	static inline double deg2rad(const double &deg) {
		return deg * DEG_TO_RAD;
	};
	static inline double rad2deg(const double &rad) {
		return rad / DEG_TO_RAD;
	}
	
};


#endif //PCD_COMPARE_GPSTOOLS_H
