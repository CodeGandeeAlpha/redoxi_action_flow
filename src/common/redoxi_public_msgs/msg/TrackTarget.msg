# empty message: track_id = -1
unique_identifier_msgs/UUID x_uid
Control x_control
# same group id means the same object
unique_identifier_msgs/UUID x_group_uid

# TODO: add track status constants

int64 track_id -1
float64 confidence
int64 track_status 0
Box2 track_bbox
Detection detection
FrameMetadata frame_metadata
AnyData any_data