#ifndef LASERSCAN_TO_RANGES_H_
#define LASERSCAN_TO_RANGES_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
//#include <laserscan_to_ranges/simple_ranges.hpp>

class LaserScanToRanges : public rclcpp::Node
{
public:
  LaserScanToRanges();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr range_pub_;
};







#endif  // LASERSCAN_TO_RANGES_H_