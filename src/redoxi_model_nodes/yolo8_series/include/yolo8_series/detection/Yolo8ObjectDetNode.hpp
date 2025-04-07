#pragma once

#include <yolo8_series/detection/Yolo8ObjectDetTypes.hpp>
#include <yolo8_series/base/Yolo8BaseNode.hpp>

namespace redoxi_works::model_nodes::yolo8
{
using Yolo8ObjectDetNode = Yolo8BaseNode<detection::InferenceModel>;
} // namespace redoxi_works::model_nodes::yolo8
