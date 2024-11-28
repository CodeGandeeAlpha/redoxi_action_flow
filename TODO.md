## Roadmap

### IMPORTANT
- in output port, "fallback_delivery_precondition" actually controls the precondition, not the downstream.precondition

### core functions
- [] support shared memory in all nodes
- [] implement python action input/output port

### platform
- [] support rk3588

### nodes
- [] general object detection node
- [] general object tracking node
- [] command line controllable video source node, set file, play, pause, stop, etc.
- [] publish-to-action node, which accepts published messages and convert them into actions, driving the video pipeline
- [] object snapshot node
- [] head detection node
- [] rtm pose node
- [] tracking node
- [] video reader by file
- [] video reader by usb camera
- [] multi stream video reader, like RGBD, stereo, etc.

### tests
- [] object detection node, including calling action (return by action result) and streaming action (send to downstream)
 


## Assumptions
[] action messages arrive in order

## Issues
[] Out-of-order reception: if node A sends (x1,x2,x3) to node B, and we are set to deliver all messages until success, then when node B receives out of order (x3,x1,x2), node A will be deadlocked, because it fails to deliver x1 and x2 and will keep retrying. The solution is to make use of the ABORTED state of the goal handle. Node B can accepts x1 if buffer allows, but because it finds x1 is out of order, it will abort it, and node A will be aware of that through result_callback.
[] Out-of-order delivery and reception: in local machine, we know there will be no message lost, so all nodes can be set to "no-lost-assumed" mode, and they will use tbb::sequencer_node to receive and send messages, using +1 increment frame number to order the messages. This only works in environment where message lost is not possible, otherwise all nodes will be deadlocked.
[] ROS2 has a timeout setting for goal handle, which is 10s by default, which can get set as follows. The timeout starts counting when the goal is accepted, so if a goal is not dealt with in time, it will be aborted. This is considered a bug, and later ROS version may fix this.(https://github.com/ros2/rcl/issues/1103)
```cpp
auto server_opt = rcl_action_server_get_default_options();
{
    std::chrono::nanoseconds timeout = std::chrono::milliseconds(300);
    server_opt.result_timeout.nanoseconds = timeout.count();
}
```
