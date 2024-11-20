#include <yolo8_body_pose_detector/Yolo8BodyPoseDetector.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <chrono>

namespace redoxi_works::model_nodes
{

Yolo8BodyPoseDetector::Yolo8BodyPoseDetector(const std::string &node_name,
                                             const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
    auto ret = declare_default_parameters_for_node(this);
    if (ret != 0) {
        RDX_RAISE_ERROR("Failed to declare default parameters for node {}", node_name);
    }

    auto node = this;
    m_json_params = RDX_GET_JSON_PARAM_FROM_NODE(node);
}

Yolo8BodyPoseDetector::~Yolo8BodyPoseDetector()
{
    // do not call stop() here, because it will NOT call the subclass's stop()
}

int Yolo8BodyPoseDetector::init(std::shared_ptr<InitConfig_t> init_config,
                                std::shared_ptr<RuntimeConfig_t> runtime_config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "initializing");

    _update_init_config(init_config);
    _update_runtime_config(runtime_config);

    return 0;
}

void Yolo8BodyPoseDetector::_update_init_config(std::shared_ptr<InitConfig_t> init_config)
{
    if (m_status != NodeStatusCode::BEFORE_INIT) {
        RDX_RAISE_ERROR("Node status must be BEFORE_INIT, but got {}", m_status);
    }

    m_init_config = init_config;

    // create input port
    m_input_port = std::make_shared<ActionInputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    // load model
    m_model = std::make_shared<inference::yolo8::Yolo8PoseModel>();
    auto model_init_params = m_model->create_init_params();
    {
        auto _init_params = std::dynamic_pointer_cast<inference::yolo8::Yolo8ModelConfig>(model_init_params);
        if (!_init_params) {
            RDX_RAISE_ERROR("Failed to cast model init params to Yolo8ModelConfig");
        }
        *_init_params = *init_config->model_config;
    }
    auto ret_model_open = m_model->open(model_init_params);
    if (ret_model_open != 0) {
        RDX_RAISE_ERROR("Failed to open model, error code: {}", ret_model_open);
    }

    // done, state change to STOPPED
    m_status = NodeStatusCode::STOPPED;
}

void Yolo8BodyPoseDetector::_update_runtime_config(std::shared_ptr<RuntimeConfig_t> runtime_config)
{
    if (m_status != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("Node status must be STOPPED, but got {}", m_status);
    }

    // state change to STARTED, then create step thread
    m_runtime_config = runtime_config;
    m_status = NodeStatusCode::STARTED;

    // create step thread, and run it
    m_step_thread = std::make_shared<std::thread>([this]() {
        while (m_status == NodeStatusCode::STARTED && rclcpp::ok()) {
            auto start_time = std::chrono::steady_clock::now();
            _step();
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < m_runtime_config->step_interval) {
                std::this_thread::sleep_for(m_runtime_config->step_interval - elapsed);
            }
        }
    });
}

void Yolo8BodyPoseDetector::_step()
{
    // do nothing if not started
    if (m_status != NodeStatusCode::STARTED) {
        return;
    }

    // get a message from input port
    std::shared_ptr<ActionInputPort_t::SourceData_t> source_data;
    auto use_blocking_mode = m_runtime_config->enable_blocking_mode;
    if (use_blocking_mode) {
        source_data = m_input_port->pop_source_data();
    } else {
        source_data = m_input_port->try_pop_source_data();
    }
    if (!source_data) {
        // no data, do nothing
        return;
    }

    // get image
    auto image = _extract_image(source_data);

    // run inference
    auto ret_inference = m_model->create_inference_inout_data();
    if (ret_inference != 0) {
        RDX_RAISE_ERROR("Failed to run inference, error code: {}", ret_inference);
    }
}

} // namespace redoxi_works::model_nodes
