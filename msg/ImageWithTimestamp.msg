# ImageWithTimestamp.msg

std_msgs/Header header      # ROS time when message is created
sensor_msgs/Image image     # The image from DAVIS
builtin_interfaces/Time img_ts  # The DAVIS frame timestamp as ROS time (converted)
