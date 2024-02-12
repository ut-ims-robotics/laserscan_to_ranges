from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  ld = LaunchDescription()

  # Declare the launch arguments
  method_arg = DeclareLaunchArgument(
    name='method',
    default_value='min',
    description='Method to use for determining range values',
    choices=['min', 'mean', 'max']
  )
  field_of_view_arg = DeclareLaunchArgument(
    name='field_of_view',
    default_value='0.0',
    description='Specifies the field of view of the sensor in degrees. ' + \
    'If 0, calculate from the LaserScan message'
  )
  angle_offset_arg = DeclareLaunchArgument(
    name='angle_offset',
    default_value='0.0',
    description='Specifies the angle offset in degrees'
  )
  enable_ranges_arg = DeclareLaunchArgument(
    name='enable_ranges',
    default_value='false',
    description='Specifies whether to publish the sensor_msgs/Range messages'
  )
  debug_arg = DeclareLaunchArgument(
    name='debug',
    default_value='false',
    description='Start the node with loglevel set to debug'
  )

  log_level = LaunchConfiguration("log_level")

  # Specify the actions
  scan_to_ranges_node = Node(
    package='laserscan_to_ranges',
    executable='laserscan_to_ranges',
    name='laserscan_to_ranges_node',
    output='screen',
    parameters=[
      {'method': LaunchConfiguration('method')},
      {'field_of_view': LaunchConfiguration('field_of_view')},
      {'angle_offset': LaunchConfiguration('angle_offset')},
      {'enable_ranges': LaunchConfiguration('enable_ranges')}
    ],
    arguments=["--ros-args", "--log-level", "laserscan_to_ranges_node:=debug"] if debug == 'true' else []
  )

  # Add the actions to the launch description
  ld.add_action(method_arg)
  ld.add_action(field_of_view_arg)
  ld.add_action(angle_offset_arg)
  ld.add_action(enable_ranges_arg)
  ld.add_action(debug_arg)
  ld.add_action(scan_to_ranges_node)

  return ld
