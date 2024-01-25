#include "laserscan_to_ranges/laserscan_to_ranges.h"
#include <algorithm>
#include <numeric>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/console.h>

LaserScanToRanges::LaserScanToRanges(): pnh_("~")
{  
  // Get the node parameters
  std::string method;
  bool debug;
  pnh_.param<std::string>("method", method, "min");
  pnh_.param<double>("field_of_view", field_of_view_, 0.0); // In degrees, 0.0 means use entire scan
  pnh_.param<double>("angle_offset", angle_offset_, 0.0);   // In degrees, positive is counter-clockwise
  pnh_.param<bool>("enable_ranges", enable_ranges_, true); // Enable publishing the standard range messages
  pnh_.param<std::string>("range_frame_prefix", range_frame_prefix_, std::string("range_")); // Prefix for the range sensor frame names
  pnh_.param<std::string>("base_frame", base_frame_, ""); // Base frame name, leave empty to use the scan frame
  pnh_.param<bool>("debug", debug, false); // Set node log level to debug

  if (debug)
  {
    ROS_INFO("Setting log level to debug");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
    ROS_DEBUG("Debug log level enabled");
  }

   std::transform(method.begin(), method.end(), method.begin(), ::tolower); // make sure string is lowercase
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
    ROS_ERROR("Invalid method parameter: '%s'", method.c_str());
  }

  // Set up the publishers and subscribers
  scan_sub_ = pnh_.subscribe("scan", 10, &LaserScanToRanges::scanCallback, this);
  simple_range_pub_ = pnh_.advertise<laserscan_to_ranges::SimpleRanges>("simple_ranges", 10);
  if (enable_ranges_)
  {
    // Initialize the range publisher and transform broadcaster
    range_pub_right_ = pnh_.advertise<sensor_msgs::Range>("range_right", 10);
    range_pub_front_ = pnh_.advertise<sensor_msgs::Range>("range_front", 10);
    range_pub_left_ = pnh_.advertise<sensor_msgs::Range>("range_left", 10);
    range_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
  }
  ROS_INFO("laserscan_to_ranges node has started");
}

void LaserScanToRanges::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  ROS_DEBUG("scan msg received: '%s'", msg->header.frame_id.c_str());

  laserscan_to_ranges::SimpleRanges simple_ranges_msg;
  simple_ranges_msg.header = msg->header;
  simple_ranges_msg.header.stamp = ros::Time::now(); // Use the current time for the header stamp
  simple_ranges_msg.right = 0;
  simple_ranges_msg.front = 0;
  simple_ranges_msg.left = 0;

  double field_of_view = field_of_view_; // In radians

  // Some sanity checks for the scan message
  // Check if the scan message is empty
  if (msg->ranges.empty())
  {
    ROS_WARN_THROTTLE(1000, "Empty scan message received");
    return;
  }

  // Check if the angle_min and angle_max are valid
  if (msg->angle_max <= msg->angle_min)
  {
    ROS_WARN_THROTTLE(1000, "Invalid angle_max or angle_min in the scan message");
    return;
  }

  // Check if the angle_increment is valid
  if (msg->angle_increment <= 0)
  {
    ROS_WARN_THROTTLE(1000, "Invalid angle_increment in the scan message");
    return;
  }

  // Check the field of view parameter
  if (field_of_view_ <= 0.0)
  {
    field_of_view = std::max((double)(msg->angle_max - msg->angle_min), 0.0); // Use entire scan
  }

  // Calculate the center index of the scan
  unsigned int center_idx = msg->ranges.size() / 2;

  // Take into account the angle offset by shifting the center index
  center_idx += angle_offset_ / msg->angle_increment;

  // Calculate the number of points per sector
  unsigned int points_per_sector = static_cast<unsigned int>(field_of_view / 3 / msg->angle_increment);

  // Calculate the start and end indices of the sectors
  auto right_start_idx = static_cast<unsigned int>(center_idx - points_per_sector * 3 / 2);
  auto front_start_idx = right_start_idx + points_per_sector;
  auto left_start_idx = front_start_idx + points_per_sector;
  auto right_end_idx = right_start_idx + points_per_sector - 1;
  auto front_end_idx = front_start_idx + points_per_sector - 1;
  auto left_end_idx = left_start_idx + points_per_sector - 1;

  // Make sure indices are within scan data bounds
  unsigned int scan_len = msg->ranges.size();
  right_start_idx = std::max(std::min(right_start_idx, scan_len - 1), 0u);
  front_start_idx = std::max(std::min(front_start_idx, scan_len - 1), 0u);
  left_start_idx = std::max(std::min(left_start_idx, scan_len - 1), 0u);
  right_end_idx = std::max(std::min(right_end_idx, scan_len - 1), 0u);
  front_end_idx = std::max(std::min(front_end_idx, scan_len - 1), 0u);
  left_end_idx = std::max(std::min(left_end_idx, scan_len - 1), 0u);

  // Calculate the range for each sector depending on the method

  // Copy the ranges for each sector into the vectors
  std::vector<double> right_ranges(msg->ranges.begin() + right_start_idx, msg->ranges.begin() + right_end_idx);
  std::vector<double> front_ranges(msg->ranges.begin() + front_start_idx, msg->ranges.begin() + front_end_idx);
  std::vector<double> left_ranges(msg->ranges.begin() + left_start_idx, msg->ranges.begin() + left_end_idx);

  // Remove NaNs from the vectors
  right_ranges.erase(std::remove_if(right_ranges.begin(), right_ranges.end(), [](double d) { return std::isnan(d); }), right_ranges.end());
  front_ranges.erase(std::remove_if(front_ranges.begin(), front_ranges.end(), [](double d) { return std::isnan(d); }), front_ranges.end());
  left_ranges.erase(std::remove_if(left_ranges.begin(), left_ranges.end(), [](double d) { return std::isnan(d); }), left_ranges.end());

  ROS_DEBUG("Points per sector: right %ld, front %ld, left %ld]", right_ranges.size(), front_ranges.size(), left_ranges.size());

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

  ROS_DEBUG("Sector values: right %f, front %f, left %f", simple_ranges_msg.right, simple_ranges_msg.front, simple_ranges_msg.left);

  // Publish the SimpleRange message
  simple_range_pub_.publish(simple_ranges_msg);

  // Publish the standard range messages if enabled
  if (enable_ranges_)
  {
    // Use the base frame if specified, otherwise use the scan frame
    std::string range_frame_id = base_frame_.empty() ? msg->header.frame_id : base_frame_;

    // Create the range messages
    sensor_msgs::Range range_msg;

    // Fill in the common fields
    range_msg.header = msg->header;
    range_msg.header.stamp = ros::Time::now(); // Use the current time for the header stamp
    range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg.field_of_view = field_of_view / 3;
    range_msg.min_range = msg->range_min;
    range_msg.max_range = msg->range_max;

    // Publish the right sector
    range_msg.header.frame_id = range_frame_id + range_frame_prefix_ + "right";
    range_msg.range = simple_ranges_msg.right;
    range_pub_right_.publish(range_msg);

    // Publish the front sector
    range_msg.header.frame_id = range_frame_id + range_frame_prefix_ + "front";
    range_msg.range = simple_ranges_msg.front;
    range_pub_front_.publish(range_msg);

    // Publish the left sector
    range_msg.header.frame_id = range_frame_id + range_frame_prefix_ + "left";
    range_msg.range = simple_ranges_msg.left;
    range_pub_left_.publish(range_msg);

    // Publish the range sensor transforms
    
    // Use the base frame if specified, otherwise use the scan frame
    std::string range_tf_frame_id = base_frame_.empty() ? msg->header.frame_id : base_frame_;
    
    tf2::Quaternion q;
    geometry_msgs::TransformStamped range_tf_msg;
    range_tf_msg.header.stamp = ros::Time::now();
    range_tf_msg.header.frame_id = range_tf_frame_id;

    // Publish the right sector transform
    range_tf_msg.child_frame_id = range_frame_prefix_ + "right";
    range_tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), -field_of_view / 3));
    range_tf_broadcaster_->sendTransform(range_tf_msg);

    // Publish the front sector transform
    range_tf_msg.child_frame_id = range_frame_prefix_ + "front";
    range_tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0));
    range_tf_broadcaster_->sendTransform(range_tf_msg);

    // Publish the left sector transform
    range_tf_msg.child_frame_id = range_frame_prefix_ + "left";
    range_tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), field_of_view / 3));
    range_tf_broadcaster_->sendTransform(range_tf_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscan_to_ranges");
  LaserScanToRanges node;
  ros::spin();
  return 0;
}
