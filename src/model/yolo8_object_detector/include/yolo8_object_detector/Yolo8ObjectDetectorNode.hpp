#pragma once

#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8DetectionModel.hpp>

#include <yolo8_object_detector/yolo8_object_detector.hpp>
#include <yolo8_object_detector/Yolo8ObjectDetectorTypes.hpp>

namespace redoxi_works::model_nodes
{

class Yolo8ObjectDetectorNode : public redoxi_works::common_nodes::StartStopNode
{
  public:
    struct ByDetectionRequest {
        using InputPort_t = yolo8_object_detector::DetectionRequestInputPort;
        using InputAction_t = typename InputPort_t::ActionType_t;
        using InputActionDataTrait_t = typename InputPort_t::ActionDataTrait_t;
        using InputGoalUUID_t = typename InputPort_t::GoalUUID_t;
        using InputSourceData_t = typename InputPort_t::SourceData_t;
    };

    struct ByImageRequest {
        using InputPort_t = yolo8_object_detector::ImageRequestInputPort;
        using InputAction_t = typename InputPort_t::ActionType_t;
        using InputActionDataTrait_t = typename InputPort_t::ActionDataTrait_t;
        using InputGoalUUID_t = typename InputPort_t::GoalUUID_t;
        using InputSourceData_t = typename InputPort_t::SourceData_t;

        using OutputPort_t = yolo8_object_detector::ImageRequestOutputPort;
        using OutputAction_t = typename OutputPort_t::ActionType_t;
        using OutputActionDataTrait_t = typename OutputPort_t::ActionDataTrait_t;
        using OutputSourceData_t = typename OutputPort_t::SourceData_t;
        using OutputRequest_t = typename OutputPort_t::DeliveryRequest_t;
    };

    using InitConfig_t = yolo8_object_detector::InitConfig;
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using RuntimeConfig_t = yolo8_object_detector::RuntimeConfig;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;
    using InferenceResource_t = yolo8_object_detector::InferenceResource;
    using InferenceModel_t = inference::yolo8::Yolo8DetectionModel;
    using DetectionResult_t = InferenceModel_t::SingleImageOutput_t;

  public:
    explicit Yolo8ObjectDetectorNode(const std::string &node_name,
                                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~Yolo8ObjectDetectorNode() noexcept;

  protected:
    // from base class
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

  protected:
    // from this class
    virtual int _extract_image(cv::Mat *output,
                               const std::shared_ptr<ByDetectionRequest::InputSourceData_t> &source_data);
};

} // namespace redoxi_works::model_nodes
