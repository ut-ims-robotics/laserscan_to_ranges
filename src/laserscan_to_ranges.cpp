#include "laserscan_to_ranges/laserscan_to_ranges.hpp"
#include "rclcpp/rclcpp.hpp"

LaserScanToRanges::LaserScanToRanges() : Node("laserscan_to_ranges")
{
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&LaserScanToRanges::scanCallback, this, std::placeholders::_1));
  //range_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("ranges", 10);
}

void LaserScanToRanges::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"I heard: '%s'", msg->header.frame_id.c_str());
  //range_pub_->publish(ranges_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanToRanges>());
  rclcpp::shutdown();
  return 0;
}
