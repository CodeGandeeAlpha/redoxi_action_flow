# empty data: len(persons) == 0
# all persons should have the same track_id, as specified by track_id
unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control
# same task uid means the same object
redoxi_public_msgs/TaskMetadata x_task_metadata

Person[] persons
int64 track_id