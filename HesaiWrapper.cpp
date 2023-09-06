#include "HesaiWrapper.hpp"

#include <filesystem>

void HesaiWrapper::lidarCallback(const LidarDecodedFrame<PointXYZIT>& frame)
{
	while (this->lock.test_and_set()) {}
	if (frame.points_num == 0) return;
	this->point_cloud->clear();
	this->point_cloud->resize(frame.points_num);
	this->point_cloud->points.assign(frame.points, frame.points + frame.points_num);
	this->point_cloud->height = 1;
	this->point_cloud->width = frame.points_num;
	this->point_cloud->is_dense = false;
	this->lock.clear();
}

HesaiWrapper::HesaiWrapper(std::string lidar_ip, std::string host_ip, int udp_port)
{
	DriverParam param;
	// assign param
	param.input_param.source_type = DATA_FROM_LIDAR;

	param.input_param.device_ip_address = lidar_ip;
	param.input_param.udp_port = 2368;
	param.input_param.host_ip_address = host_ip;
	param.input_param.multicast_ip_address = "";

	this->start(param);
}

HesaiWrapper::HesaiWrapper(std::string pcap_path, std::string correction_file_path, std::string firetimes_path)
{
	DriverParam param;
	// assign param

	if (!std::filesystem::exists(pcap_path)) {
		std::cerr << "pcap file does not exist: " << pcap_path << std::endl;
		return;
	}

	param.input_param.source_type = DATA_FROM_PCAP;

	param.input_param.pcap_path = pcap_path;
	param.input_param.correction_file_path = correction_file_path;
	param.input_param.firetimes_path = firetimes_path;

	this->start(param);
}

void HesaiWrapper::start(DriverParam param)
{
	this->lidar_sdk = std::make_unique<HesaiLidarSdk<PointXYZIT>>();
	this->point_cloud = std::make_unique<pcl::PointCloud<PointXYZIT>>();
	this->writer = std::make_unique<pcl::PCDWriter>();

	//init lidar with param
	if(!this->lidar_sdk->Init(param))
	{
		std::cerr << "Exiting before" << std::endl;
		return;
	}

	//assign callback fuction
	this->lidar_sdk->RegRecvCallback(std::bind(&HesaiWrapper::lidarCallback, this, std::placeholders::_1));

	//star process thread
	this->lidar_sdk->Start();

	std::cout << "setting started true" << std::endl;
	this->started = true;
}

std::string HesaiWrapper::get_point_cloud()
{
	if (!std::filesystem::is_directory("/tmp/HesaiWrapper") || !std::filesystem::exists("/tmp/HesaiWrapper")) { // Check if src folder exists
		std::filesystem::create_directory("/tmp/HesaiWrapper"); // create src folder
	}
	while (this->lock.test_and_set()) {}

	std::string file_name = "/tmp/HesaiWrapper/PointCloud" + std::to_string(this->count++) + ".pcd";
	writer->writeASCII(file_name, *this->point_cloud);

	lock.clear();

	return file_name;
}

bool HesaiWrapper::is_started()
{
	return this->started;
}

HesaiWrapper::~HesaiWrapper()
{
	 this->lidar_sdk->Stop();
}
