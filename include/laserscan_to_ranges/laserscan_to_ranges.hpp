#ifndef LASERSCAN_TO_RANGES_H_
#define LASERSCAN_TO_RANGES_H_

#include <rclcpp/rclcpp.hpp>
#include <laserscan_to_ranges/msg/simple_ranges.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/transform_broadcaster.h>

class LaserScanToRanges : public rclcpp::Node
{
public:
  LaserScanToRanges();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<laserscan_to_ranges::msg::SimpleRanges>::SharedPtr simple_range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_front_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_left_;
  
  // Node params
  enum class Method
  {
    MIN,
    MAX,
    MEAN
  };
  Method method_;
  double field_of_view_;
  double angle_offset_;
  bool enable_ranges_;
  std::string range_frame_prefix_;
  std::string base_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> range_tf_broadcaster_;
};

#endif  // LASERSCAN_TO_RANGES_H_