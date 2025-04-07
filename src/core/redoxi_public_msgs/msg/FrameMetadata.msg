# empty data: frame_num == -1
unique_identifier_msgs/UUID x_uid
Control x_control
std_msgs/Header header

# this is a logical frame number, not the actual frame number from the video source
# it usually starts from 0, and increases by 1 for each frame
# frame number, -1 for empty frame
int64 frame_num -1

# this records timestamp of the frame in video source
builtin_interfaces/Time source_timestamp

# this records the frame index in video source
# -1 for empty value
int64 source_frame_index -1

# encoding format of the frame data, e.g., 'rgb8', 'bgr8', etc.
# see sensor_msgs/image_encodings for more details
# if not specified, the application makes its own decision
string encoding

# image size
int64 width 0
int64 height 0

# about the source of this frame
FrameDeviceTag device_tag

# any other data
AnyData any_data
