#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>

class LowPassFilter : public rclcpp::Node
{
public:
  LowPassFilter() : Node("low_pass_filter"), alpha_(0.2), filtered_value_(0.0), 
                   is_initialized_(false), count_(0)
  {
    subscription_ = create_subscription<std_msgs::msg::Float64>(
      "raw_data", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        
        double raw_value = msg->data;
        
        double new_filtered_value = applyLowPassFilter(raw_value);

        auto filtered_msg = std_msgs::msg::Float64();
        filtered_msg.data = new_filtered_value;
        filtered_publisher_->publish(filtered_msg);
        
        auto point_msg = geometry_msgs::msg::PointStamped();
        point_msg.header.stamp = this->now();
        point_msg.header.frame_id = "lowpass_filtered";
        point_msg.point.x = count_ * 0.1;
        point_msg.point.y = new_filtered_value;
        point_msg.point.z = raw_value;
        point_publisher_->publish(point_msg);
        
        RCLCPP_INFO(get_logger(), "Low-pass: raw=%.4f, filtered=%.4f", 
                   raw_value, new_filtered_value);
        
        count_++;
      });
    
    filtered_publisher_ = create_publisher<std_msgs::msg::Float64>("lowpass_filtered_data", 10);
    point_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("lowpass_filtered_point", 10);
  }

private:
  double applyLowPassFilter(double raw_value) {
    if (!is_initialized_) {
      filtered_value_ = raw_value;
      is_initialized_ = true;
    } else {
      filtered_value_ = alpha_ * raw_value + (1.0 - alpha_) * filtered_value_;
    }
    return filtered_value_;
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;
  double alpha_;
  double filtered_value_;
  bool is_initialized_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowPassFilter>());
  rclcpp::shutdown();
  return 0;
}