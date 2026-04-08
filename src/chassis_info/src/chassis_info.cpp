#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

class ChassisInfo : public rclcpp::Node
{
public:
  ChassisInfo() : Node("chassis_info"), v_(0.0), yaw_(0.0)
  {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        v_ = msg->linear.x*330.0;
        yaw_ = msg->angular.z*0.001;
      });

    chassis_info_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/chassis_info", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ChassisInfo::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.linear.x = v_;
      msg.linear.y = yaw_;
    }
    chassis_info_pub_->publish(msg);
  }

  double v_;
  double yaw_;
  std::mutex mutex_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_info_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisInfo>());
  rclcpp::shutdown();
  return 0;
}
