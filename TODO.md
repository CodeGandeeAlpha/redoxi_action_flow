## Roadmap

### IMPORTANT
- when using shared memory, shm data may be dropped in the middle of the pipeline (timeout dropping), nodes need to handle this (got an image but cannot read it).
- in output port, "fallback_delivery_precondition" actually controls the precondition, not the downstream.

### known bugs
- [] probe message uuid is always 0, not initialized by source data?

### performance optimization
- [] group multiple nodes into a single process
    - [] use ros2 lifecycle nodes to do external initialization
- [] for nodes within a single process, transmit device memory directly (rk3588, cuda, etc.)
- [] use boost shared memory to transfer frames

### core functions
- [] python action input/output port
- [] batched processing in port handlers

### video source node
- [] support filelist as input, automatic switch between files
- [] support frame skipping
- [] accept piped video from ffmpeg
- [] accept rtsp stream
- [] accept usb camera
- [] subscribe to a topic, and convert it to action
- [] collect and transmit IMU sensor data and camera pose data (like ios arframe)
- [] command line controllable video source node, set file, play, pause, stop, etc.

### platform
- [] Nvidia Orin
- [x] support rk3588

### other nodes
- [] SPEC human pose node
- [x] general object detection node
- [x] general object tracking node

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
