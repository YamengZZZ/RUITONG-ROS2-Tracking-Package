//RT SDK header
#include "ARMDCombinedAPI.h"

//ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include "ruitong_demo/msg/tool_tracking_msg.hpp"
#include "ruitong_demo/msg/tracking_data_msg.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

class RuitongTrackingNode : public rclcpp::Node
{
public:
	RuitongTrackingNode() : Node("ruitong_tracking_node")
	{
		// Declare parameters
		this->declare_parameter("hostname", "RT-MAX000752.local");
		this->declare_parameter("tool_path", "/home/yameng/ros2_ws/src/RUITONG-Demo/mytool");
		
		// Get parameters
		hostname_ = this->get_parameter("hostname").as_string();
		tool_path_ = this->get_parameter("tool_path").as_string();
		
		// If tool_path is empty, use package share directory
		if (tool_path_.empty())
		{
			try {
				std::string pkg_dir = ament_index_cpp::get_package_share_directory("ruitong_demo");
				tool_path_ = pkg_dir + "/tool";
			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "Failed to get package directory: %s", e.what());
				rclcpp::shutdown();
				return;
			}
		}
		
		RCLCPP_INFO(this->get_logger(), "Tool path: %s", tool_path_.c_str());
		
		// Create publisher with larger queue for 120Hz publishing
		tracking_pub_ = this->create_publisher<ruitong_demo::msg::TrackingDataMsg>(
			"tracking_data", 100);
		
		// Initialize tracker
		m_Tracker_ = new ARMDCombinedAPI();
		
		// Create tool directory if it doesn't exist
		if (!fs::exists(tool_path_))
		{
			RCLCPP_INFO(this->get_logger(), "Creating tool directory: %s", tool_path_.c_str());
			try {
				fs::create_directories(tool_path_);
			} catch (const std::exception& e) {
				RCLCPP_ERROR(this->get_logger(), "Failed to create tool directory: %s", e.what());
				delete m_Tracker_;
				rclcpp::shutdown();
				return;
			}
		}

		// If there are no existing .arom files in tool_path_, generate one.
		bool has_arom_pre = false;
		if (fs::exists(tool_path_) && fs::is_directory(tool_path_))
		{
			for (const auto& entry : fs::directory_iterator(tool_path_))
			{
				if (entry.path().extension() == ".arom")
				{
					has_arom_pre = true;
					break;
				}
			}
		}
		if (!has_arom_pre)
		{
			RCLCPP_INFO(this->get_logger(), "No .arom files in %s — generating default AROM", tool_path_.c_str());
			generateAROMfile();
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Found existing .arom files in %s — skipping generation", tool_path_.c_str());
		}
		
		// Connect to device
		if (connect(hostname_) != 0)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to connect to device");
			delete m_Tracker_;
			rclcpp::shutdown();
			return;
		}
		
		// Load tools - check if AROM files exist
		bool has_arom_files = false;
		if (fs::exists(tool_path_) && fs::is_directory(tool_path_))
		{
			for (const auto& entry : fs::directory_iterator(tool_path_))
			{
				if (entry.path().extension() == ".arom")
				{
					has_arom_files = true;
					break;
				}
			}
		}
		
		if (!has_arom_files)
		{
			RCLCPP_ERROR(this->get_logger(), "No AROM files found in: %s", tool_path_.c_str());
			RCLCPP_ERROR(this->get_logger(), "Make sure AROM files exist in the tool directory");
			m_Tracker_->disconnect();
			delete m_Tracker_;
			rclcpp::shutdown();
			return;
		}
		
		m_Tracker_->loadPassiveToolAROM(tool_path_);
		RCLCPP_INFO(this->get_logger(), "Tools loaded successfully");
		
		// Start tracking
		m_Tracker_->startTracking();
		m_Tracker_->startImaging();
		
		// Create timer for 60Hz publishing
		timer_ = this->create_wall_timer(
			std::chrono::microseconds(16667),  // 1/60 Hz ≈ 16.667 ms
			std::bind(&RuitongTrackingNode::publishTrackingData, this));
		
		RCLCPP_INFO(this->get_logger(), "Ruitong tracking node started - publishing at 60 Hz");
	}
	
	~RuitongTrackingNode()
	{
		closeSystem();
	}

private:
	void generateAROMfile()
	{
		vector<MarkerPosition> markers;
		MarkerPosition a, b, c, d;
		
		a.P[0] = 49.629; a.P[1] = 0.0; a.P[2] = 0.0; a.P[3] = 1.0;
		b.P[0] = 0.0; b.P[1] = 0.0; b.P[2] = 0.0; b.P[3] = 1.0;
		c.P[0] = -54.165; c.P[1] = 22.940; c.P[2] = 0.0; c.P[3] = 1.0;
		d.P[0] = -83.050; d.P[1] = -20.844; d.P[2] = -0.167; d.P[3] = 1.0;
		
		markers.push_back(a);
		markers.push_back(b);
		markers.push_back(c);
		markers.push_back(d);
		
		ToolCalibrationData newTool;
		// name must be alphanumeric (letters and numbers) to satisfy generateAROM checks
		newTool.name = "mytool";
		newTool.type = 0;
		newTool.planeNum = 1;
		newTool.minNumMarker = 3;
		newTool.calbError = -1;
		newTool.markers.push_back(markers);
		newTool.pin[0] = 0.0;
		newTool.pin[1] = 0.0;
		newTool.pin[2] = 0.0;
		
		// Use tool_path_ with trailing slash
		std::string arom_path = tool_path_;
		if (!arom_path.empty() && arom_path.back() != '/')
			arom_path += "/";
		
		RCLCPP_INFO(this->get_logger(), "AROM generation: %s", 
			GenerationStatus::toString(m_Tracker_->generateAROM(arom_path, newTool)).c_str());
	}
	
	int connect(string hostname)
	{
		RCLCPP_INFO(this->get_logger(), "Connecting to device: %s", hostname.c_str());
		
		int errorCode = m_Tracker_->connect(hostname);
		if (errorCode == 0)
		{
			RCLCPP_INFO(this->get_logger(), "Connected successfully");
			RCLCPP_INFO(this->get_logger(), "Local IP: %s", 
				m_Tracker_->getConnectionIPs()[0].c_str());
			RCLCPP_INFO(this->get_logger(), "Device IP: %s", 
				m_Tracker_->getConnectionIPs()[1].c_str());
			return 0;
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", 
				ConnectionStatus::toString(m_Tracker_->getConnectionStatus()).c_str());
			return -1;
		}
	}
	
	void closeSystem()
	{
		if (m_Tracker_ != nullptr)
		{
			if (ConnectionStatus::Interruption != m_Tracker_->getConnectionStatus())
			{
				m_Tracker_->stopTracking();
				m_Tracker_->stopImaging();
				m_Tracker_->disconnect();
			}
			delete m_Tracker_;
			m_Tracker_ = nullptr;
		}
		RCLCPP_INFO(this->get_logger(), "System closed");
	}
	
	void publishTrackingData()
	{
		// Check for system alerts
		if (DeviceAlert::Normal != m_Tracker_->getSystemAlert())
		{
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
				"Device alert: %s", 
				DeviceAlert::toString(m_Tracker_->getSystemAlert()).c_str());
		}
		
		// Check for connection interruption
		if (ConnectionStatus::Interruption == m_Tracker_->getConnectionStatus())
		{
			RCLCPP_ERROR(this->get_logger(), "Connection interrupted");
			rclcpp::shutdown();
			return;
		}
		
		// Update tracking data
		m_Tracker_->trackingUpdate();
		
		// Create message
		auto msg = ruitong_demo::msg::TrackingDataMsg();
		msg.header.stamp = this->now();
		msg.header.frame_id = "ruitong_tracker";
		
		// Get gravity vector
		std::vector<double> gravity = m_Tracker_->getGravityVector();
		msg.gravity_vector[0] = gravity.at(0);
		msg.gravity_vector[1] = gravity.at(1);
		msg.gravity_vector[2] = gravity.at(2);
		
		// Get all markers
		vector<MarkerPosition> allMarkerData = m_Tracker_->getAllMarkers();
		msg.markers_number = allMarkerData.size();
		
		// Get tracking data
		vector<ToolTrackingData> toolData = m_Tracker_->getTrackingData(allMarkerData);
		
		// Get stray markers
		msg.stray_markers_number = m_Tracker_->getUnMatchedMarkers().size();
		
		// Process each tool
		for (const auto& tool : toolData)
		{
			auto tool_msg = ruitong_demo::msg::ToolTrackingMsg();
			
			tool_msg.tool_name = tool.name;
			tool_msg.transformation_status = TransformationStatus::toString(tool.transform.status);
			tool_msg.match_status = tool.matchStatus;
			
			if (tool.matchStatus)
			{
				// Marker positions
				for (const auto& marker : tool.markers)
				{
					tool_msg.marker_positions.push_back(marker.P[0]);
					tool_msg.marker_positions.push_back(marker.P[1]);
					tool_msg.marker_positions.push_back(marker.P[2]);
				}
				
				// Acquisition time
				tool_msg.acquisition_time = tool.timespec;
				
				// Matching error
				tool_msg.matching_error = tool.transform.error;
				
				// Transformation matrix (4x4, row-major)
				for (int j = 0; j < 4; j++)
				{
					for (int k = 0; k < 4; k++)
					{
						tool_msg.transformation_matrix[j * 4 + k] = tool.transform.matrix[j][k];
					}
				}
				
				// Tip position
				tool_msg.tip_position[0] = tool.transform.tx;
				tool_msg.tip_position[1] = tool.transform.ty;
				tool_msg.tip_position[2] = tool.transform.tz;
			}
			
			msg.tools.push_back(tool_msg);
		}
		
		// Publish message
		tracking_pub_->publish(msg);
	}

private:
	ARMDCombinedAPI* m_Tracker_;
	rclcpp::Publisher<ruitong_demo::msg::TrackingDataMsg>::SharedPtr tracking_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	string hostname_;
	string tool_path_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RuitongTrackingNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
