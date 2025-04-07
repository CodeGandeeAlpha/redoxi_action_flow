#pragma once

// precompiled headers
#include <filesystem>
#include <redoxi_common_cpp/_pch.hpp>
#include <redoxi_common_nodes/_pch.hpp>
#include <spdlog/spdlog.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <yolo8_series/base/Yolo8BaseTypes.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>
#include <redoxi_dnn_models/message_conversion.hpp>
#include <redoxi_dnn_models/visualizations.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessReplyHandler.hpp>