# empty message: category = -1
unique_identifier_msgs/UUID x_uid
Control x_control
# same group id means the same object
unique_identifier_msgs/UUID x_group_uid

int64 category -1
string category_name
float64 confidence 0.0
SemanticIdentity semantic_identity
Box2 bbox
float64[] feature
# bool is_detected_by_camera  # TODO: remove this
Keypoints keypoints
Frame mask
FrameMetadata frame_metadata
AnyData any_data
