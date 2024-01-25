#ifndef LASERSCAN_TO_RANGES_H_
#define LASERSCAN_TO_RANGES_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laserscan_to_ranges/SimpleRanges.h>
#include <sensor_msgs/Range.h>
#include <tf2_ros/transform_broadcaster.h>

class LaserScanToRanges
{
public:
  LaserScanToRanges();

private:
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  ros::Subscriber scan_sub_;        // Subscriber to laser scan
  ros::Publisher simple_range_pub_; // Publisher to simple range
  ros::Publisher range_pub_right_;  // Publisher to right range
  ros::Publisher range_pub_front_;  // Publisher to front range
  ros::Publisher range_pub_left_;   // Publisher to left range
  ros::NodeHandle pnh_;             // Private nodehandle

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

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> range_tf_broadcaster_;
};

#endif  // LASERSCAN_TO_RANGES_H_
