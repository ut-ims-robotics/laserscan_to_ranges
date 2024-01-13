# laserscan\_to\_ranges
The laserscan\_to\_ranges package offers simplification and intuitive representation of the LaserScan messages.
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# Overview

The package subscribes to a LaserScan data and splits it into three equal sectors: left, front, and right. Each sector is describe by only a single floating point value calculation of which can be specified via a parameter. By default, the closest point of the given sector will be used. The package is useful for building simple navigation and obstacle avoidance algorithms.

The resulting data is published as a custom SimpleRanges message containing the following fields:
* `left`: range of the left sector
* `front`: range of the front sector
* `right`: range of the right sector

# Parameters

The package has the following parameters:
* `method` (`string`, default: `min`) - specifies the method for the range calculation. The following options are available:
  * `min`: the closest distance in the sector
  * `mean`: the mean distance in the sector
  * `max`: the farthest distance in the sector
* `field_of_view` (`double`, default: `0.0`) - specifies the field of view of the sensor in degrees. If the value is set to `0.0`, the field of view will be calculated from the `angle_min` and `angle_max` fields of the LaserScan message.
* `angle_offset` (`double`, default: `0.0`) - specifies the angle offset in degrees. The offset is added to the angle_min and angle_max of the LaserScan messages. For example, if the sensors field of view is [-90, 90] degrees and the offset is 10 degrees, the 60 degree front sector will be shifted from [-30, 30] to [-20, 40].


# Running
To run the node, simply enter:<br/>
```ros2 run laserscan_to_ranges laserscan_to_ranges```

or use the launch file:<br/>
```ros2 launch laserscan_to_ranges laserscan_to_ranges.launch.py```
