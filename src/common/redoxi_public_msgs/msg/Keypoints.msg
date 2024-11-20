# empty message : len(x) == 0
# x, y is relative to bbox, in aboslute unit (xmax==bbox.width, ymax==bbox.height)
# bbox is relative to frame, in pixel unit
unique_identifier_msgs/UUID x_uid
Control x_control
# same group id means the same object
unique_identifier_msgs/UUID x_group_uid

float32[] confidence
geometry_msgs/Point[] keypoints_2
# nx2 array representing edges between keypoints, (u1,v1,u2,v2,...)
# u1,v1 are indices of keypoints, u2,v2 are indices of keypoints
int64[] keypoint_edges

int64[] semantic_type
FrameMetadata frame_metadata
AnyData any_data