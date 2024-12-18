# empty message: track_id = -1
unique_identifier_msgs/UUID x_uid
Control x_control
# same task uid means the same object
redoxi_public_msgs/TaskMetadata x_task_metadata

# bitmask of track status

# TODO: add track status constants

# id of this track target
int64 track_id -1

# long-term identity of this track target
# multiple track_id may have the same long_term_identity
int64 long_term_identity 0

# track status of an object, it is a bitmask
# use & to check if a status is set
TrackObjectStatus track_status

# algorithm-selected bbox of the track target, maybe from true_detection or predicted_detection
# if you want to choose by yourself, look at true_detection or predicted_detection
Box2 track_bbox

# confidence of the track_bbox
float64 confidence

# detection from image, may be empty
Detection true_detection

# detection by tracker using motion prediction or other methods, may be empty
Detection predicted_detection

# metadata of the frame
FrameMetadata frame_metadata

# any additional data
AnyData any_data