# empty data: frame_num == -1
unique_identifier_msgs/UUID x_uid
Control x_control
# same group id means the same object
unique_identifier_msgs/UUID x_group_uid

FrameMetadata metadata

# cache data in external storage (such as shared memory)
SharedMemoryToken shm_token

# raw data, mainly for debugging, usually empty
sensor_msgs/Image raw_image
sensor_msgs/CompressedImage encoded_image

# any other data
AnyData any_data
