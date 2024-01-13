#ifndef LASERSCAN_TO_RANGES_H_
#define LASERSCAN_TO_RANGES_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laserscan_to_ranges/msg/simple_ranges.hpp>

class LaserScanToRanges : public rclcpp::Node
{
public:
  LaserScanToRanges();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<laserscan_to_ranges::msg::SimpleRanges>::SharedPtr range_pub_;
  
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
};







#endif  // LASERSCAN_TO_RANGES_H_