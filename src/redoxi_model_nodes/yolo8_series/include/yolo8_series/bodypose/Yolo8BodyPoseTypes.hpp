#pragma once

#include <yolo8_series/base/Yolo8BaseTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>

namespace redoxi_works::model_nodes::yolo8::bodypose
{
using InferenceModel = redoxi_works::inference::yolo8::Yolo8PoseModel;
using InferenceResource = redoxi_works::model_nodes::yolo8::InferenceResource<InferenceModel>;
using InitConfig = redoxi_works::model_nodes::yolo8::InitConfig<InferenceModel>;
using RuntimeConfig = redoxi_works::model_nodes::yolo8::RuntimeConfig<InferenceModel>;

} // namespace redoxi_works::model_nodes::yolo8::bodypose
