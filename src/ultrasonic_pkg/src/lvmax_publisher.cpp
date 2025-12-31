#include <iostream>
#include <string>
#include <atomic>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

const char USB_PORT[] = "/dev/ttyUSB1";
int arduino;

class LVMAX_Publisher : public rclcpp::Node
{
  public:
    LVMAX_Publisher () : Node ("lvmax_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("ultrasonic_data", 10);
      fd_ = open (USB_PORT, O_RDONLY);
      if (fd_ < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", USB_PORT, strerror(errno));
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Successfully opened %s", USB_PORT);

      auto timer_callback = 
        [this]() -> void {
          if (fd_ < 0)
          {
            auto message = std_msgs::msg::String();
            message.data = "SERIAL_PORT_ERROR";
            publisher_->publish(message);
            RCLCPP_WARN(this->get_logger(), "Serial port not available");
            return;
          }
          
          char buf[128];
          memset(buf, 0, sizeof(buf));

          ssize_t n;
          int i = 0;
          while (i < 127)
          {
            n = read(fd_, (buf + i * sizeof(char)), 1);
            if (n < 0)
              return;

            char* databyte = static_cast<char*>(buf);
            if (databyte[i] == '\n')
            {
              i++;
              break;
            }

            i++;
          }

          if (i < 2)
            return;

          buf[i] = '\0';
          auto message = std_msgs::msg::String();
          message.data = std::string(buf);
          publisher_->publish(message);
          RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());

        };

      timer_ = this->create_wall_timer (500ms, timer_callback);
    }

    ~LVMAX_Publisher() {
      if (fd_ >= 0)
        close(fd_);
    }
      
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int fd_;
};

int main (int argc, char * argv[])
{
  rclcpp::init (argc, argv);
  auto node = std::make_shared<LVMAX_Publisher>();
  rclcpp::spin (node);
  rclcpp::shutdown ();

  return 0;
}