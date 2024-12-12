# the goal part of ProcessTrackedObjects.action

unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control
redoxi_public_msgs/TaskMetadata x_task_metadata

# the tracker response
# the tracked objects returned by the tracker
redoxi_public_msgs/TrackTarget[] track_targets

# optional: the current frame
redoxi_public_msgs/MultiDeviceFrame frame_bundle
