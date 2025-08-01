# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

**Primary build command:**
```bash
./xbuild_working_nodes.sh [--verbose] [--debug|--release]
```
- Uses colcon to build specific ROS2 packages with optimized parallel workers
- Default build type is RelWithDebInfo; use `--debug` or `--release` to override
- Automatically adjusts parallel jobs based on available memory
- Sources the install/setup.bash after building

**Clean build:**
```bash
./xclean_build_install.sh
```
- Removes build/, install/, and log/ directories completely

**Other build modes:**
- `--verbose`: Enable detailed build output with `--event-handlers console_direct+`
- Memory-aware: Automatically reduces parallel jobs if less than 4GB RAM available

## Dependencies and Setup

**Required dependencies (install in order):**
1. Latest ROS2 installation
2. ONNX: `scripts/onnx-download.bash`
3. Vineyard (v6d): Scripts in `scripts/v6d/`
4. YOLO8 models: `scripts/yolo8-onnx-download.bash`

**Key external dependencies:**
- Clang/Clang++ (set as default compiler)
- TBB (Intel Threading Building Blocks)
- JSON Struct library with specific flags
- OpenCV and FFmpeg for video processing
- Shared memory systems (v6d/Vineyard)

## Architecture Overview

**Core Structure:**
- `src/core/`: Foundational libraries and utilities
- `src/models/`: ML model implementations (YOLO8, MOT trackers)  
- `src/examples/`: Example applications and pipelines
- `src/extern/`: External dependencies and submodules

**Key Architectural Patterns:**
- **Node-based pipeline architecture**: Components are ROS2 lifecycle nodes that can be chained
- **Port-based communication**: Async input/output ports for data flow between nodes
- **Action-based processing**: Uses ROS2 actions for request/response patterns
- **Shared memory optimization**: Uses Vineyard (v6d) for zero-copy data sharing
- **Template-heavy design**: Extensive use of C++20 concepts and templates

**Core Libraries:**
- `redoxi_common_nodes`: Base node classes, port abstractions, lifecycle management
- `redoxi_common_cpp`: Utility classes, concepts, async processors
- `redoxi_public_msgs`: ROS2 message definitions for frames, detections, tracking
- `redoxi_video_reader`: FFmpeg-based video input nodes
- `redoxi_inference_*`: Inference backends (ONNX, RKNN)

**Pipeline Examples:**
- Video → Detection: `video_detection_pipeline.cpp`
- Video → Detection + Tracking: Available in launch files
- Simple video playback: `simple_video_reader_node.cpp`

## Development Patterns

**C++ Standards (from .cursorrules):**
- Use `//!` for single-line Doxygen comments instead of block comments
- Use `__func__` instead of `__FUNCTION__` for function names
- C++20 standard with concepts and templates extensively used

**Configuration Pattern:**
- Nodes use InitConfig/RuntimeConfig template pattern
- JSON Struct library for configuration serialization
- Parameter parsing through BaseRosNode infrastructure

**Memory Management:**
- Shared memory via Vineyard for large data (images, tensors)
- Expiration-based caching for memory cleanup
- Token-based shared memory references in messages

## Testing and Examples

**Run examples:**
```bash
# After building, choose either:
# Launch files (in src/examples/redoxi_example_cpp/launch/):
ros2 launch redoxi_example_cpp lch_simple_video_reader_node.py
ros2 launch redoxi_example_cpp lch_video_detection_pipeline.py

# Or direct executables:
ros2 run redoxi_example_cpp simple_video_reader_node
ros2 run redoxi_example_cpp video_detection_pipeline
```

**Test package:** `redoxi_cpp_tests` for unit testing

## Environment Setup

**Required environment variables:**
- CUDA_VISIBLE_DEVICES=0 (automatically set in WSL2)
- Various ROS2 domain and DDS configuration scripts available in `scripts/`

**Logging:**
- Uses spdlog with configurable log levels via compile-time macros
- Log output directory: `log/` with timestamped subdirectories