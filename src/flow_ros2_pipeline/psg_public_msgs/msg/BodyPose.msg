# empty message : len(x) == 0
# x, y is relative to bbox, in aboslute unit (xmax==bbox.width, ymax==bbox.height)
# bbox is relative to frame, in pixel unit
unique_identifier_msgs/UUID uuid
float32[] confidence
geometry_msgs/Point[] keypoints_2
int64[] semantic_type
Frame frame
Box2 bbox