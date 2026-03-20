#include <iostream>
#include <string>
#include <atomic>
#include <chrono>
#include <cstring>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class RobotGPSPublisher : public rclcpp::Node
{
	public:
		RobotGPSPublisher()
		: Node("robot_gps_publisher")
		{
			if (setup_gps_temp() != 0)
			{
				RCLCPP_INFO(this->get_logger(), "Failed to initialize robot GPS.\n");
				return;
			}
			
			publisher_ = this->create_publisher<sensor_msgs::msg::Range>(
				"robot_gps", 10
			);
			frame_ = "robot_gps_frame";
			param_ = "robot_gps_frame_id";
			this->declare_parameter<std::string>(param_, frame_);
			std::string frame_id_ = this->get_parameter(param_).as_string();
			
			status_.status = sensor_msgs::msg::NavSatStatus::FIX;
			status_.service = sensor_msgs::msg::NavSatStatus::GPS;
			
			message_.header.frame_id = frame_id;
			
			// @todo: Do we need / have this information?
			message_.position_covariance = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
			message_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
			
			timer_ = this->create_wall_timer(
				200ms, std::bind(&RobotGPSPublisher::timer_callback, this)
			);
		}
		
		// @todo
		int setup_gps_temp (void)
		{
			return 0;
		}
		
		// @todo
		void get_data_temp (float64& lat, float64& long, float64& alt)
		{
			return;
		}
		
		// @todo
		int get_status_temp (void)
		{
			return 0;
		}
		
	private:
	
		void
		timer_callback()
		{
			get_data_temp (latitude_, longitude_, altitude_);
			
			status_.status = get_status_temp ();
			if (status_.status == sensor_msgs::msg::NavSatStatus::NO_FIX)
			{
				return;
			}
			message_.header.stamp = this->get_clock()->now();
			message_.latitude = latitude_;
			message_.longitude = longitude_;
			message_.altitude = altitude_;
			
			
		}
	
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
		sensor_msgs::msg::NavSatFix message_;
		sensor_msgs::msg::NavSatStatus status_;
		float64 latitude_, longitude_, altitude_;
		
		rclcpp::TimerBase::SharedPtr timer_;		
		std::string frame_, param_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world gps_pkg package\n");
  return 0;
}
