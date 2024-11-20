#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <nlohmann/json.hpp>

#include <yolo8_body_pose_detector/yolo8_body_pose_detector.hpp>
#include <yolo8_body_pose_detector/Yolo8BodyPoseDetectorTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>


namespace redoxi_works::model_nodes
{

class Yolo8BodyPoseDetector : public rclcpp::Node,
                              public IStartStopProtocol
{
  public:
    using ActionInputPort_t = yolo8_body_pose_detector::DetectionActionInputPort;
    using InputAction_t = typename ActionInputPort_t::ActionType_t;
    using InitConfig_t = yolo8_body_pose_detector::InitConfig;
    using RuntimeConfig_t = yolo8_body_pose_detector::RuntimeConfig;

  public:
    explicit Yolo8BodyPoseDetector(const std::string &node_name,
                                   const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~Yolo8BodyPoseDetector();

    int init(std::shared_ptr<InitConfig_t> init_config,
             std::shared_ptr<RuntimeConfig_t> runtime_config);

    // IStartStopProtocol interface
    int start();
    int stop();

    const auto &get_json_params() const
    {
        return m_json_params;
    }

  protected:
    virtual void _step();
    virtual void _update_init_config(std::shared_ptr<InitConfig_t> init_config);
    virtual void _update_runtime_config(std::shared_ptr<RuntimeConfig_t> runtime_config);
    cv::Mat _extract_image(const std::shared_ptr<ActionInputPort_t::SourceData_t> &source_data);

  protected:
    std::shared_ptr<ActionInputPort_t> m_input_port;
    std::atomic<int> m_status = NodeStatusCode::BEFORE_INIT;

    std::shared_ptr<InitConfig_t> m_init_config;
    std::shared_ptr<RuntimeConfig_t> m_runtime_config;

    std::shared_ptr<std::thread> m_step_thread;
    std::shared_ptr<inference::yolo8::Yolo8PoseModel> m_model;

    inference::InferenceInOutData::Ptr m_inference_inout_data;
    nlohmann::json m_json_params;
};

} // namespace redoxi_works::model_nodes
