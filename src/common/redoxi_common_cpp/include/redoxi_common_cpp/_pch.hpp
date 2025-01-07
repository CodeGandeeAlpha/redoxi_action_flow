#pragma once

// precompiled headers
#include <string>
#include <numeric>
#include <optional>
#include <chrono>
#include <vector>
#include <memory>
#include <cstddef>
#include <concepts>
#include <atomic>
#include <limits>
#include <cstdint>
#include <any>
#include <variant>
#include <unordered_set>
#include <map>
#include <unordered_map>

#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging_macros.h>
#include <rcpputils/asserts.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <fmt/format.h>
#include <tbb/tbb.h>
#include <nlohmann/json.hpp>
#include <json_struct/json_struct.h>
#include <opencv2/opencv.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
