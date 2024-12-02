#pragma once

#include <yolo8_series/base/Yolo8BaseTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8DetectionModel.hpp>

namespace redoxi_works::model_nodes::yolo8::detection
{
using InferenceModel = redoxi_works::inference::yolo8::Yolo8DetectionModel;
using InferenceResource = InferenceResource<InferenceModel>;
using InitConfig = InitConfig<InferenceModel>;
using RuntimeConfig = RuntimeConfig<InferenceModel>;
} // namespace redoxi_works::model_nodes::yolo8::detection
