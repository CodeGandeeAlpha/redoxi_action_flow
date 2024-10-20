# empty data: frame_num == -1
unique_identifier_msgs/UUID uuid
Control x_control
std_msgs/Header header

# cache data in external storage (such as shared memory)
CacheData cache

# raw data, mainly for debugging, usually empty
sensor_msgs/Image raw_image
sensor_msgs/CompressedImage encoded_image

# frame number, -1 for empty frame
int64 frame_num -1

# deprecated, use x_control.code instead
int64 signal_code 0