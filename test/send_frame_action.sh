#!/bin/bash

id_int="$1"
action_name="$2"
action_type="$3"
frame_num="$4"
x_uid="$5"

# create a multiline string which is a json
# store into variable named goal_msg
goal_msg='{
    "frame": {
      "cache": {
        "id_int": '"$id_int"',
        "has_int_id": 1
      },
      "frame_num": '"$frame_num"'
    },
    "x_uid": {
      "uuid": '"$x_uid"'
    }
  }'

echo "Sending goal message: $goal_msg"


ros2 action send_goal /"$action_name" psg_actions/action/"$action_type" "$goal_msg"