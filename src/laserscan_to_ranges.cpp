#include "laserscan_to_ranges/laserscan_to_ranges.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

LaserScanToRanges::LaserScanToRanges() : Node("laserscan_to_ranges")
{
  // Read in parameters
  this->declare_parameter("method", "min");
  this->declare_parameter("field_of_view", 0.0); // In degrees, 0.0 means use entire scan
  this->declare_parameter("angle_offset", 0.0);  // In degrees, positive is counter-clockwise
  this->declare_parameter("enable_ranges", true); // Enable publishing the standard range messages
  this->declare_parameter("range_frame_prefix", "range_"); // Prefix for the range sensor frame names
  this->declare_parameter("base_frame", ""); // Base frame name, leave empty to use the scan frame
  std::string method = this->get_parameter("method").as_string();
  std::transform(method.begin(), method.end(), method.begin(), ::tolower);// make sure string is lowercase
  if (method == "min")
  {
    method_ = Method::MIN;
  }
  else if (method == "max")
  {
    method_ = Method::MAX;
  }
  else if (method == "mean")
  {
    method_ = Method::MEAN;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(),"Invalid method parameter: '%s'", method.c_str());
  }
  // Read in the field of view and angle offset parameters and convert both to radians
  field_of_view_ = this->get_parameter("field_of_view").as_double() * M_PI / 180.0;
  angle_offset_ = this->get_parameter("angle_offset").as_double() * M_PI / 180.0;
  enable_ranges_ = this->get_parameter("enable_ranges").as_bool();
  range_frame_prefix_ = this->get_parameter("range_frame_prefix").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  // Set up the publishers and subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&LaserScanToRanges::scanCallback, this, std::placeholders::_1));
  simple_range_pub_ = this->create_publisher<laserscan_to_ranges::msg::SimpleRanges>("simple_ranges", 10);
  if (enable_ranges_)
  {
    // Initialize the range publisher and transform broadcaster
    range_pub_right_ = this->create_publisher<sensor_msgs::msg::Range>("range_right", 10);
    range_pub_front_ = this->create_publisher<sensor_msgs::msg::Range>("range_front", 10);
    range_pub_left_ = this->create_publisher<sensor_msgs::msg::Range>("range_left", 10);
    range_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

}

void LaserScanToRanges::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(),"scan msg received: '%s'", msg->header.frame_id.c_str());

  auto simple_ranges_msg = laserscan_to_ranges::msg::SimpleRanges();
  simple_ranges_msg.header = msg->header;
  simple_ranges_msg.header.stamp = this->get_clock()->now(); // Use the current time for the header stamp
  simple_ranges_msg.right = 0;
  simple_ranges_msg.front = 0;
  simple_ranges_msg.left = 0;

  double field_of_view = field_of_view_; // In radians
  
  // Some sanity checks for the scan message
  // Check if the scan message is empty
  if (msg->ranges.empty())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty scan message received");
    return;
  }

  // Check if the angle_min and angle_max are valid
  if (msg->angle_max <= msg->angle_min)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid angle_max or angle_min in the scan message");
    return;
  }

  // Check if the angle_increment is valid
  if (msg->angle_increment <= 0)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid angle_increment in the scan message");
    return;
  }

  // Check the field of view parameter
  if (field_of_view_ <= 0.0)
  {
    field_of_view = std::max(msg->angle_max - msg->angle_min, 0.0f); // Use entire scan
  }
  
  // Calculate the center index of the scan
  unsigned int center_idx = msg->ranges.size()/2;
  
  // Take into account the angle offset by shifting the center index
  center_idx += angle_offset_/msg->angle_increment;
  
  // Calculate the number of points per sector
  unsigned int points_per_sector = (unsigned int)(field_of_view / 3 / msg->angle_increment);
  

  // Calculate the start and end indices of the sectors
  auto right_start_idx = (unsigned int)(center_idx - points_per_sector*3/2);
  auto front_start_idx = right_start_idx + points_per_sector;
  auto left_start_idx = front_start_idx + points_per_sector;
  auto right_end_idx = right_start_idx + points_per_sector - 1;
  auto front_end_idx = front_start_idx + points_per_sector - 1;
  auto left_end_idx = left_start_idx + points_per_sector - 1;
  
  // Make sure indices are within scan data bounds
  unsigned int scan_len = msg->ranges.size(); 
  right_start_idx = std::max(std::min(right_start_idx, scan_len-1), 0u);
  front_start_idx = std::max(std::min(front_start_idx, scan_len-1), 0u);
  left_start_idx = std::max(std::min(left_start_idx, scan_len-1), 0u);
  right_end_idx = std::max(std::min(right_end_idx, scan_len-1), 0u);
  front_end_idx = std::max(std::min(front_end_idx, scan_len-1), 0u);
  left_end_idx = std::max(std::min(left_end_idx, scan_len-1), 0u);
  
  // Calculate the range for each sector depending on the method
  
  
  // Copy the ranges for each sector into the vectors
  std::vector<double> right_ranges(msg->ranges.begin() + right_start_idx, msg->ranges.begin() + right_end_idx);
  std::vector<double> front_ranges(msg->ranges.begin() + front_start_idx, msg->ranges.begin() + front_end_idx);
  std::vector<double> left_ranges(msg->ranges.begin() + left_start_idx, msg->ranges.begin() + left_end_idx);

  // Remove NaNs from the vectors
  right_ranges.erase(std::remove_if(right_ranges.begin(), right_ranges.end(), [](double d){return std::isnan(d);}), right_ranges.end());
  front_ranges.erase(std::remove_if(front_ranges.begin(), front_ranges.end(), [](double d){return std::isnan(d);}), front_ranges.end());
  left_ranges.erase(std::remove_if(left_ranges.begin(), left_ranges.end(), [](double d){return std::isnan(d);}), left_ranges.end());

  RCLCPP_INFO(this->get_logger(), "right %ld, front %ld, left %ld", right_ranges.size(), front_ranges.size(), left_ranges.size());

  switch (method_)
  {
    case Method::MIN:
      // Find the minimum range for each sector, if the sector is empty set the range to infinity
      simple_ranges_msg.right = right_ranges.empty() ? std::numeric_limits<double>::infinity() : *std::min_element(right_ranges.begin(), right_ranges.end());
      simple_ranges_msg.front = front_ranges.empty() ? std::numeric_limits<double>::infinity() : *std::min_element(front_ranges.begin(), front_ranges.end());
      simple_ranges_msg.left = left_ranges.empty() ? std::numeric_limits<double>::infinity() : *std::min_element(left_ranges.begin(), left_ranges.end());
      break;
    case Method::MAX:
      // Find the maximum range for each sector, if the sector is empty set the range to 0
      simple_ranges_msg.right = right_ranges.empty() ? 0 : *std::max_element(right_ranges.begin(), right_ranges.end());
      simple_ranges_msg.front = front_ranges.empty() ? 0 : *std::max_element(front_ranges.begin(), front_ranges.end());
      simple_ranges_msg.left = left_ranges.empty() ? 0 : *std::max_element(left_ranges.begin(), left_ranges.end());
      break;
    case Method::MEAN:
      // Find the mean range for each sector, if the sector is empty set the range to infinity
      simple_ranges_msg.right = right_ranges.empty() ? std::numeric_limits<double>::infinity() : std::accumulate(right_ranges.begin(), right_ranges.end(), 0.0) / right_ranges.size();
      simple_ranges_msg.front = front_ranges.empty() ? std::numeric_limits<double>::infinity() : std::accumulate(front_ranges.begin(), front_ranges.end(), 0.0) / front_ranges.size();
      simple_ranges_msg.left = left_ranges.empty() ? std::numeric_limits<double>::infinity() : std::accumulate(left_ranges.begin(), left_ranges.end(), 0.0) / left_ranges.size();
      break;
  }

  RCLCPP_INFO(this->get_logger(), "right %f, front %f, left %f", simple_ranges_msg.right, simple_ranges_msg.front, simple_ranges_msg.left);

  // Publish the SimpleRange message
  simple_range_pub_->publish(simple_ranges_msg);

  // Publish the standard range messages if enabled
  if (enable_ranges_)
  {
    // Use the base frame if specified, otherwise use the scan frame
    geometry_msgs::msg::TransformStamped range_tf_msg_;
    if (base_frame_.empty())
    {
      range_tf_msg_.header.frame_id = msg->header.frame_id;
    }
    else
    {
      range_tf_msg_.header.frame_id = base_frame_;
    }

    // Create the range messages
    auto range_msg = sensor_msgs::msg::Range();

    // Fill in the common fields
    range_msg.header = msg->header;
    range_msg.header.stamp = this->get_clock()->now(); // Use the current time for the header stamp
    range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range_msg.field_of_view = field_of_view / 3;
    range_msg.min_range = msg->range_min;
    range_msg.max_range = msg->range_max;

    // Publish the right sector
    range_msg.header.frame_id = range_frame_prefix_ + "right";
    range_msg.range = simple_ranges_msg.right;
    range_pub_right_->publish(range_msg);

    // Publish the front sector
    range_msg.header.frame_id = range_frame_prefix_ + "front";
    range_msg.range = simple_ranges_msg.front;
    range_pub_front_->publish(range_msg);

    // Publish the left sector
    range_msg.header.frame_id = range_frame_prefix_ + "left";
    range_msg.range = simple_ranges_msg.left;
    range_pub_left_->publish(range_msg);

    // Prepare the transform messages
    tf2::Quaternion q;

    range_tf_msg_.header.stamp = msg->header.stamp;
    range_tf_msg_.child_frame_id = range_frame_prefix_ + "right";
    q = tf2::Quaternion(tf2::Vector3(0, 0, 1), -field_of_view / 3);
    range_tf_msg_.transform.rotation = tf2::toMsg(q);
    range_tf_broadcaster_->sendTransform(range_tf_msg_);

    range_tf_msg_.child_frame_id = range_frame_prefix_ + "front";
    q = tf2::Quaternion(tf2::Vector3(0, 0, 1), 0);
    range_tf_msg_.transform.rotation = tf2::toMsg(q);
    range_tf_broadcaster_->sendTransform(range_tf_msg_);

    range_tf_msg_.child_frame_id = range_frame_prefix_ + "left";
    q = tf2::Quaternion(tf2::Vector3(0, 0, 1), field_of_view / 3);
    range_tf_msg_.transform.rotation = tf2::toMsg(q);
    range_tf_broadcaster_->sendTransform(range_tf_msg_);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanToRanges>());
  rclcpp::shutdown();
  return 0;
}
