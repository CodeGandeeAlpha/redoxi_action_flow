#!/bin/bash

id_int="$1"
action_name="$2"
action_type="$3"

# create a multiline string which is a json
# store into variable named goal_msg
goal_msg='{
    "frame": {
      "cache": {
        "id_int": '"$id_int"',
        "has_int_id": 1
      },
      "frame_num": 0
    },
    "detections_uuid": {
      "uuid": [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
    }
  }'

echo "Sending goal message: $goal_msg"


ros2 action send_goal /"$action_name" psg_actions/action/"$action_type" "$goal_msg"