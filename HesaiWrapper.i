%module hesai
%include "std_string.i"

%{
#include "HesaiWrapper.hpp"
%}

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