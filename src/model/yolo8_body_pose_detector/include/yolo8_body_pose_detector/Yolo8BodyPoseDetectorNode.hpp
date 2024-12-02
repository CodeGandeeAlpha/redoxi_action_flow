#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <nlohmann/json.hpp>

#include <yolo8_body_pose_detector/yolo8_body_pose_detector.hpp>
#include <yolo8_body_pose_detector/Yolo8BodyPoseDetectorTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>

#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <sensor_msgs/image_encodings.hpp>


namespace redoxi_works::model_nodes
{

class Yolo8BodyPoseDetectorNode : public redoxi_works::common_nodes::StartStopNode
{
  public:
    inline static constexpr const char *RequiredImageEncoding = sensor_msgs::image_encodings::RGB8;

  public:
    struct ByDetectionRequest {
        using InputPort_t = yolo8_body_pose_detector::DetectionRequestInputPort;
        using InputAction_t = typename InputPort_t::ActionType_t;
        using InputActionDataTrait_t = typename InputPort_t::ActionDataTrait_t;
        using InputGoalUUID_t = typename InputPort_t::GoalUUID_t;
        using InputSourceData_t = typename InputPort_t::SourceData_t;
    };

    struct ByImageRequest {
        using InputPort_t = yolo8_body_pose_detector::ImageRequestInputPort;
        using InputAction_t = typename InputPort_t::ActionType_t;
        using InputActionDataTrait_t = typename InputPort_t::ActionDataTrait_t;
        using InputGoalUUID_t = typename InputPort_t::GoalUUID_t;
        using InputSourceData_t = typename InputPort_t::SourceData_t;

        using OutputPort_t = yolo8_body_pose_detector::ImageRequestOutputPort;
        using OutputAction_t = typename OutputPort_t::ActionType_t;
        using OutputActionDataTrait_t = typename OutputPort_t::ActionDataTrait_t;
        using OutputSourceData_t = typename OutputPort_t::SourceData_t;
        using OutputRequest_t = typename OutputPort_t::DeliveryRequest_t;
    };

    using InitConfig_t = yolo8_body_pose_detector::InitConfig;
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using RuntimeConfig_t = yolo8_body_pose_detector::RuntimeConfig;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;
    using InferenceResource_t = yolo8_body_pose_detector::InferenceResource;
    using InferenceModel_t = inference::yolo8::Yolo8PoseModel;
    using DetectionResult_t = InferenceModel_t::SingleImageOutput_t;

  public:
    explicit Yolo8BodyPoseDetectorNode(const std::string &node_name,
                                       const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~Yolo8BodyPoseDetectorNode() noexcept;

  protected:
    // from base class
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

  protected:
    // from this class
    virtual int _extract_image(cv::Mat *output, const std::shared_ptr<ByDetectionRequest::InputSourceData_t> &source_data);

    virtual int _process_detection_request();

    //! create a new pull process reply handler for detection request
    virtual int _create_detection_request_handler(const RuntimeConfig_t &runtime_config);

    //! extract image from source data
    virtual int _extract_image(cv::Mat *output, const std::shared_ptr<ByImageRequest::InputSourceData_t> &source_data);

    //! create a new pull process send handler for image request
    virtual int _create_image_request_handler(const RuntimeConfig_t &runtime_config);

    //! process image request
    virtual int _process_image_request();

    //! Draw visualization on canvas
    virtual void _draw_visualization(cv::Mat &canvas,
                                     const DetectionResult_t &detections);

    //! create a new inference resource, and push it to the concurrent queue
    //! @param replicas: number of replicas to create, replicated resource will share the same model but with different inout data
    //! @return 0 if success, -1 if failed
    virtual int _create_inference_resource(InitConfig_t::ModelConfig_t::Ptr model_config, int replicas = 1);
    virtual int _create_all_inference_resources(const std::vector<InitConfig_t::ModelConfig_t::Ptr> &model_configs);


  protected:
    std::shared_ptr<ByDetectionRequest::InputPort_t> m_detection_request_input_port;
    std::shared_ptr<ByImageRequest::InputPort_t> m_image_request_input_port;
    std::shared_ptr<ByImageRequest::OutputPort_t> m_image_request_output_port;

  private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;

    int _extract_image(cv::Mat *output, const redoxi_public_msgs::msg::Frame &frame_msg);
    int _do_inference(DetectionResult_t *output_result,
                      const cv::Mat &input_image,
                      const InferenceResource_t &resource,
                      std::optional<UUIDType> msg_uuid = std::nullopt);
    void _close_all_ports();
};

} // namespace redoxi_works::model_nodes
