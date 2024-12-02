#pragma once

#include <yolo8_series/bodypose/Yolo8BodyPoseTypes.hpp>
#include <yolo8_series/base/Yolo8BaseNode.hpp>

namespace redoxi_works::model_nodes::yolo8
{
using Yolo8BodyPoseNode = Yolo8BaseNode<bodypose::InferenceModel>;
} // namespace redoxi_works::model_nodes::yolo8
