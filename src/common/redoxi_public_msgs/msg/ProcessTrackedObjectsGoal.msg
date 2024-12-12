# the goal part of ProcessTrackedObjects.action

unique_identifier_msgs/UUID x_uid
Control x_control
TaskMetadata x_task_metadata

# the tracker response
# the tracked objects returned by the tracker
TrackTarget[] track_targets

# optional: the current frame
MultiDeviceFrame frame_bundle
