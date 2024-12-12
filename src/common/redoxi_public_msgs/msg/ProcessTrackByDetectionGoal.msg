# the goal part of ProcessTrackByDetection.action

unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control
redoxi_public_msgs/TaskMetadata x_task_metadata

# input detections and frame, required
redoxi_public_msgs/Detection[] detections
redoxi_public_msgs/MultiDeviceFrame frame_bundle
