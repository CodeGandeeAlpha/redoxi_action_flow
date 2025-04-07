# The Redoxi video analysis framework, based on ROS2

## Installation

- install the latest ros2
- checkout all code and submodules
- install onnx, using `scripts/onnx-download.bash`
- install v6d, using scripts in `scripts/v6d`
- download yolo8 models, using `scripts/yolo8-onnx-download.bash`
- build with `xbuild_working_nodes.sh`
- run the examples in `redoxi_example_cpp`

Currently the documentation is lacking, so you may need to read the CMakeLists.txt files of the packages to understand how to build things.

Basically, if any package is missing, please install it via apt, pip, rosdep or other ways.

## Examples

you can either run the examples using plain c++ or launch files, they are NOT dependent on each other.

### Using launch files
- `lch_simple_video_reader_node.py`: a simple example to read a video and publish the frames, just a single node, no downstream.
- `lch_simple_video_readout_pipeline.py`: a simple example to read a video and send to downstream nodes.
- `lch_video_detection_pipeline.py`: a more complex example that shows how to connect multiple nodes to form a pipeline.

### Using plain c++
- `simple_video_reader_node.cpp`: a simple example to read a video and publish the frames, just a single node, no downstream. 
- `simple_video_readout_pipeline.cpp`: a simple example to read a video and send to downstream nodes.
- `video_detection_pipeline.cpp`: a more complex example that shows how to connect multiple nodes to form a pipeline.

