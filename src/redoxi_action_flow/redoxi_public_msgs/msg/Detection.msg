# empty message: category = -1
unique_identifier_msgs/UUID x_uid
Control x_control
# same task uid means the same object
redoxi_public_msgs/TaskMetadata x_task_metadata

int64 category -1
string category_name
float64 confidence 0.0
SemanticIdentity semantic_identity
Box2 bbox
float64[] feature
Keypoints keypoints
Frame mask
FrameMetadata frame_metadata
AnyData any_data
