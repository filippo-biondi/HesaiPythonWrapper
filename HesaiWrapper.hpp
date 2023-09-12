#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "hesai_lidar_sdk.hpp"
#include "driver_param.h"

struct PointXYZIT {
	PCL_ADD_POINT4D
	float intensity;
	double timestamp;
	uint16_t ring;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
		PointXYZIT,
		(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
		 double, timestamp, timestamp)(uint16_t, ring, ring)
)

class HesaiWrapper
{
private:
	std::unique_ptr<HesaiLidarSdk<PointXYZIT>> lidar_sdk;
	std::unique_ptr<pcl::PointCloud<PointXYZIT>> point_cloud;
	std::atomic_flag lock = ATOMIC_FLAG_INIT;
	int count = 0;
	std::unique_ptr<pcl::PCDWriter> writer;
	bool started = false;

	void lidarCallback(const LidarDecodedFrame<PointXYZIT>& frame);
	void start(DriverParam param);

public:
	HesaiWrapper(std::string lidar_ip, std::string host_ip, int udp_port);
	HesaiWrapper(std::string pcap_path, std::string correction_file_path, std::string firetimes_path);
	~HesaiWrapper();
	std::string get_point_cloud();
	bool is_started();
};
