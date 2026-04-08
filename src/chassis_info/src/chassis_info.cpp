#include <geometry_msgs/msg/twist_stamped.hpp>
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
        v_ = msg->linear.x;
        yaw_ = msg->angular.z;
      });

    chassis_info_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("/chassis_info", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ChassisInfo::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.twist.linear.x = v_;
      msg.twist.linear.y = yaw_;
    }
    chassis_info_pub_->publish(msg);
  }

  double v_;
  double yaw_;
  std::mutex mutex_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_info_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisInfo>());
  rclcpp::shutdown();
  return 0;
}
