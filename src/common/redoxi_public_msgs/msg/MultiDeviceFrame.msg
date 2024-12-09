# a bunch of frames from multiple devices (parts of the same device)
unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control
redoxi_public_msgs/TaskMetadata x_task_metadata

# the frame from primary device
# when refer to "a frame from this device", it means the primary_frame
Frame primary_frame

# the frame from other subordinate devices
Frame[] secondary_frames
