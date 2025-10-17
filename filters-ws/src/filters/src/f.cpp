#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include <random>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class LogDataPublisher : public rclcpp::Node
{
public:
  LogDataPublisher() : Node("log_data_publisher"), count_(0)
  {
    raw_publisher_ = create_publisher<std_msgs::msg::Float64>("raw_data", 10);
    point_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("log_point", 10);
    
    random_engine_ = std::default_random_engine(std::random_device{}());
    noise_distribution_ = std::normal_distribution<double>(0.0, 0.1);
    
    timer_ = create_wall_timer(500ms, [this]() {
      auto now = this->now();

      double x = count_ * 0.1;
      double log_value = std::log(x + 1.0);
      
      double noise = noise_distribution_(random_engine_);
      double noisy_value = log_value + noise;
      

      auto raw_msg = std_msgs::msg::Float64();
      raw_msg.data = noisy_value;
      raw_publisher_->publish(raw_msg);
      
      auto point_msg = geometry_msgs::msg::PointStamped();
      point_msg.header.stamp = now;
      point_msg.header.frame_id = "log_data";
      point_msg.point.x = x;          
      point_msg.point.y = noisy_value; 
      point_msg.point.z = log_value; 
      point_publisher_->publish(point_msg);
      
      RCLCPP_INFO(get_logger(), "Publishing: x=%.2f, raw=%.4f, true=%.4f", 
                 x, noisy_value, log_value);
      
      count_++;
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr raw_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;
  size_t count_;
  std::default_random_engine random_engine_;
  std::normal_distribution<double> noise_distribution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LogDataPublisher>());
  rclcpp::shutdown();
  return 0;
}