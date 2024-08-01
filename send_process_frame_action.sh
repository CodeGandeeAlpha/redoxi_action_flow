#!/bin/bash

# create a multiline string which is a json
# store into variable named goal_msg
goal_msg='{
    "frame": {
      "cache": {
        "id_int": 4215320581763318,
        "has_int_id": 1
      },
      "frame_num": 0
    },
    "detections_uuid": {
      "uuid": [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
    }
  }'


ros2 action send_goal /model_process_frame_action psg_actions/action/ProcessFrame "$goal_msg"