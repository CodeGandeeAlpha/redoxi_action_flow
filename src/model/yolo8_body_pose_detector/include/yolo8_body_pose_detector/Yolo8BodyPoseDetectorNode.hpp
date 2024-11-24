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
    using ActionInputPort_t = yolo8_body_pose_detector::DetectionRequestInputPort;
    using InputAction_t = typename ActionInputPort_t::ActionType_t;
    using ActionDataTrait_t = typename ActionInputPort_t::ActionDataTrait_t;
    using InitConfig_t = yolo8_body_pose_detector::InitConfig;
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using RuntimeConfig_t = yolo8_body_pose_detector::RuntimeConfig;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;
    using GoalUUID_t = ActionInputPort_t::GoalUUID_t;
    using SourceData_t = ActionInputPort_t::SourceData_t;
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
    int _extract_image(cv::Mat *output, const std::shared_ptr<ActionInputPort_t::SourceData_t> &source_data);

    //! Draw visualization on canvas
    void _draw_visualization(cv::Mat &canvas,
                             const DetectionResult_t &detections);

    //! create a new inference resource, and push it to the concurrent queue
    //! @param replicas: number of replicas to create, replicated resource will share the same model but with different inout data
    //! @return 0 if success, -1 if failed
    int _create_inference_resource(InitConfig_t::ModelConfig_t::Ptr model_config, int replicas = 1);
    int _create_all_inference_resources(const std::vector<InitConfig_t::ModelConfig_t::Ptr> &model_configs);

    // void _register_input_port_callbacks(std::shared_ptr<ActionInputPort_t> input_port);
  protected:
    std::shared_ptr<ActionInputPort_t> m_input_port;

  private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
};

} // namespace redoxi_works::model_nodes
