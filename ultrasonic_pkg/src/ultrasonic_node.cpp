#include <csignal>
#include <string>
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ultrasonic_pkg/lidar/RPLidar.hpp"

bool running=true; //Variable to manage stops when signals are received
void signal_handler(int signo){
	if(signo==SIGTERM || signo==SIGINT)
		running = false;
}

class UltrasonicPublisher : public rclcpp::Node
{
	public:
		UltrasonicPublisher (int argc, char** argv) : Node("ultrasonic_publisher")
		{
			running_ = this->lidar_.init(argc>1?argv[argc - 1]:"/dev/ttyUSB0");
			publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
			
			auto timer_callback =
				[this]() -> void {
					auto message = std_msgs::msg::String();

				};
			timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);

		}

		void GetLidarStatus ()
		{
			this->lidar_.print_status();
		}

		bool IsRunning ()
		{
			return this->running_;
		}

		void RunLidar ()
		{
			this->running_ = this->lidar_.start();
			while (this->running_)
			{
				this->lidar_.update();
				this->lidar_.print_scan();
				this->lidar_.print_deltas();
			}
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
		RPLidar lidar_;
		bool running_;
};


int main(int argc, char** argv){
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UltrasonicPublisher>(argc, argv));
	rclcpp::shutdown();

	return 0;
}
