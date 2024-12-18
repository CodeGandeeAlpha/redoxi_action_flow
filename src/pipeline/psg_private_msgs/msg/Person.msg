unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control
# same task uid means the same object
redoxi_public_msgs/TaskMetadata x_task_metadata

# true_xxx means the detection from image
# xxx means the detection from tracker prediction
redoxi_public_msgs/Detection head
redoxi_public_msgs/Detection true_head
redoxi_public_msgs/Detection body
redoxi_public_msgs/Detection true_body
redoxi_public_msgs/Detection face
redoxi_public_msgs/Detection true_face

redoxi_public_msgs/FrameMetadata frame_metadata
int64 track_id
float64 body_height
float64 body_height_conf
geometry_msgs/Point head_position_3
geometry_msgs/Point foot_position_3