#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <algorithm>
#include <chrono>

class MedianFilter : public rclcpp::Node
{
public:
  MedianFilter() : Node("median_filter"), window_size_(5), count_(0)
  {
    subscription_ = create_subscription<std_msgs::msg::Float64>(
      "raw_data", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        
        double raw_value = msg->data;
        
        data_window_.push_back(raw_value);
        if (data_window_.size() > window_size_) {
          data_window_.erase(data_window_.begin());
        }

        double median_value = raw_value;
        if (data_window_.size() == window_size_) {
          median_value = applyMedianFilter();
        }
        
        auto filtered_msg = std_msgs::msg::Float64();
        filtered_msg.data = median_value;
        filtered_publisher_->publish(filtered_msg);
        
        auto point_msg = geometry_msgs::msg::PointStamped();
        point_msg.header.stamp = this->now();
        point_msg.header.frame_id = "median_filtered";
        point_msg.point.x = count_ * 0.1;
        point_msg.point.y = median_value;
        point_msg.point.z = raw_value;
        point_publisher_->publish(point_msg);
        
        RCLCPP_INFO(get_logger(), "Median: raw=%.4f, filtered=%.4f", 
                   raw_value, median_value);
        
        count_++;
      });
    
    filtered_publisher_ = create_publisher<std_msgs::msg::Float64>("median_filtered_data", 10);
    point_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("median_filtered_point", 10);
  }

private:
  double applyMedianFilter() {
    std::vector<double> sorted_data = data_window_;
    std::sort(sorted_data.begin(), sorted_data.end());
    return sorted_data[window_size_ / 2];
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_publisher_;
  std::vector<double> data_window_;
  const int window_size_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MedianFilter>());
  rclcpp::shutdown();
  return 0;
}