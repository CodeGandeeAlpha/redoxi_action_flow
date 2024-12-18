# see PassengerFlow/include/PassengerFlow/Scene/EventZoneData.h for detail
unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control
# same task uid means the same object
redoxi_public_msgs/TaskMetadata x_task_metadata

int64 event_type
string event_zone_name
float32 start_time
float32 end_time
int64 track_id
string event_pattern
string matched_trajectory
geometry_msgs/Vector3 speed_2  # speed in image space, in pixel unit