#pragma once

// precompiled header
#include <redoxi_common_cpp/_pch.hpp>

#include <functional>
#include <atomic>
#include <future>
#include <typeinfo>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_queue.h>
#include <boost/uuid/uuid_io.hpp>
#include <boost/thread/synchronized_value.hpp>

#include <optional>
#include <chrono>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <json_struct/json_struct.h>
#include <nlohmann/json.hpp>
