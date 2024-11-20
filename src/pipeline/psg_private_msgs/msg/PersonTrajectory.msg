# empty data: len(persons) == 0
# all persons should have the same track_id, as specified by track_id
unique_identifier_msgs/UUID x_uid
redoxi_public_msgs/Control x_control
# same group id means the same object
unique_identifier_msgs/UUID x_group_uid

Person[] persons
int64 track_id