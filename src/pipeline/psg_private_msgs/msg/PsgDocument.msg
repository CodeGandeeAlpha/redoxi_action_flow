# constants
string BODY_DET="body_detections"
string HEAD_DET="head_detections"
string FACE_DET="face_detections"
string BODY_POSE="body_poses"
string HEAD_POSE="head_poses"
string FACE_POSE="face_poses"

unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control

unique_identifier_msgs/UUID uuid
unique_identifier_msgs/UUID detections_uuid
# unique_identifier_msgs/UUID poses_uuid
# unique_identifier_msgs/UUID trajectories_uuid

std_msgs/Header header
Persons persons
redoxi_public_msgs/Detections body_detections
redoxi_public_msgs/Detections head_detections
redoxi_public_msgs/Detections face_detections
redoxi_public_msgs/Keypoints[] body_keypoints
redoxi_public_msgs/Keypoints[] head_keypoints
redoxi_public_msgs/Keypoints[] face_keypoints
# nx2 array representing correspondence between detections, between keypoints, or between detections and keypoints
# element is the type of the element, using constants above
string[] match_pairs

PersonTrajectories trajectories
TrajectoryEvents trajectory_events
redoxi_public_msgs/Frame frame