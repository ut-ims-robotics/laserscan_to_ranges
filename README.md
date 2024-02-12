# laserscan\_to\_ranges
The laserscan\_to\_ranges package offers simplification and intuitive representation of the LaserScan messages.
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# Overview

The package subscribes to a LaserScan data and splits it into three equal sectors: left, front, and right. Each sector is describe by only a single floating point value calculation of which can be specified via a parameter. By default, the closest point of the given sector will be used. The package is useful for building simple navigation and obstacle avoidance algorithms.

The resulting data is published as a custom SimpleRanges message containing the following fields:
* `left`: range of the left sector
* `front`: range of the front sector
* `right`: range of the right sector

# ROS API
## Subscribed topics

* `~/scan` ([sensor\_msgs/LaserScan](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)) - the LaserScan data

## Published topics

* `~/simple_ranges` ([laserscan\_to\_ranges/SimpleRanges](msg/SimpleRanges.msg)) - the simplified LaserScan data
* Optional topics (enabled via the `enable_ranges` parameter):
  * `~/ranges_left` ([sensor\_msgs/Range](https://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)) - the left sector range
  * `~/ranges_front` ([sensor\_msgs/Range](https://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)) - the front sector range
  * `~/ranges_right` ([sensor\_msgs/Range](https://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html)) - the right sector range

## Parameters

The package has the following parameters:
* `method` (`string`, default: `min`) - specifies the method for the range calculation. The following options are available:
  * `min`: the closest distance in the sector
  * `mean`: the mean distance in the sector
  * `max`: the farthest distance in the sector
* `field_of_view` (`double`, default: `0.0`) - specifies the field of view of the sensor in degrees. If the value is set to `0.0`, the field of view will be calculated from the `angle_min` and `angle_max` fields of the LaserScan message.
* `angle_offset` (`double`, default: `0.0`) - specifies the angle offset in degrees. The offset is added to the angle_min and angle_max of the LaserScan messages. For example, if the sensors field of view is [-90, 90] degrees and the offset is 10 degrees, the 60 degree front sector will be shifted from [-30, 30] to [-20, 40].
* `enable_ranges` (`bool`, default: `false`) - By default the node publish only SimpleRanges messages on the `simple_ranges` topic. Enable this parameter to additionally publish the [sensor_msgs/Range](https://docs.ros.org/en/api/sensor_msgs/html/msg/Range.html) messages on the `ranges_right`, `ranges_front`, and `ranges_left` topics. Transforms for each sector are also published. This option is useful for visualization the ranges in RViz.
* `debug` (`bool`, default: `false`) - enable this parameter to start the node with log level set to `DEBUG`.


# Running
To run the node, simply enter:<br/>
```ros2 run laserscan_to_ranges laserscan_to_ranges```

To run the node with parameters, enter:<br/>
```ros2 run laserscan_to_ranges laserscan_to_ranges --ros-args -p enable_ranges -p method:=mean -p debug:=true```

or use the launch file:<br/>
```ros2 launch laserscan_to_ranges laserscan_to_ranges.launch.py --ros-args -p enable_ranges -p method:=mean -p debug:=true```
