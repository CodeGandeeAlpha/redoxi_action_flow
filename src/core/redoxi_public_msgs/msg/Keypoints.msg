# empty message : len(x) == 0
# x, y is relative to bbox, in aboslute unit (xmax==bbox.width, ymax==bbox.height)
# bbox is relative to frame, in pixel unit
unique_identifier_msgs/UUID x_uid
Control x_control
# same task uid means the same object
redoxi_public_msgs/TaskMetadata x_task_metadata

# 2d keypoints in image coordinate
float32[] confidence
geometry_msgs/Point[] keypoints_2

# 3d keypoints in world coordinate or camera coordinate
float32[] confidence_3
geometry_msgs/Point[] keypoints_3

# nx2 array representing edges between keypoints, (u1,v1,u2,v2,...)
# u1,v1 are indices of keypoints, u2,v2 are indices of keypoints
int64[] keypoint_edges

int64[] semantic_type
FrameMetadata frame_metadata
AnyData any_data