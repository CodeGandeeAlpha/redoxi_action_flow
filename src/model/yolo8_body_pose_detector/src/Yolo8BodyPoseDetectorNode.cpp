#include <chrono>
#include <map>

#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>
#include <boost/thread/synchronized_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <yolo8_body_pose_detector/Yolo8BodyPoseDetectorNode.hpp>
#include <yolo8_body_pose_detector/Yolo8BodyPoseDetectorImpl.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_dnn_models/message_conversion.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <std_msgs/msg/string.hpp>

using DetectionMessage_t = redoxi_public_msgs::msg::Detection;
using PointMessage_t = geometry_msgs::msg::Point;

namespace redoxi_works::model_nodes
{

Yolo8BodyPoseDetectorNode::Yolo8BodyPoseDetectorNode(const std::string &node_name,
                                                     const rclcpp::NodeOptions &options)
    : redoxi_works::common_nodes::StartStopNode(node_name, options)
{
    m_impl = std::make_shared<Impl>();
}

Yolo8BodyPoseDetectorNode::~Yolo8BodyPoseDetectorNode() noexcept
{
    _close_all_ports();
}

int Yolo8BodyPoseDetectorNode::_start()
{
    // start the input port
    if (m_detection_request_input_port) {
        auto ret = m_detection_request_input_port->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to start detection request input port, error code: {}", __func__, ret);
        }
    }
    if (m_image_request_input_port) {
        auto ret = m_image_request_input_port->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to start image request input port, error code: {}", __func__, ret);
        }
    }
    if (m_image_request_output_port) {
        auto ret = m_image_request_output_port->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to start image request output port, error code: {}", __func__, ret);
        }
    }
    return 0;
}

int Yolo8BodyPoseDetectorNode::_extract_image(cv::Mat *output,
                                              const std::shared_ptr<ByDetectionRequest::InputSourceData_t> &source_data)
{
    if (!output) {
        //! nothing to do
        return 0;
    }

    return _extract_image(output, source_data->get_goal()->frame);
}

int Yolo8BodyPoseDetectorNode::_extract_image(cv::Mat *output,
                                              const std::shared_ptr<ByImageRequest::InputSourceData_t> &source_data)
{
    if (!output) {
        //! nothing to do
        return 0;
    }

    return _extract_image(output, source_data->get_goal()->frame);
}

int Yolo8BodyPoseDetectorNode::_extract_image(cv::Mat *output,
                                              const redoxi_public_msgs::msg::Frame &frame_msg)
{
    if (!output) {
        //! nothing to do
        return 0;
    }

    //! TODO: currently only support raw image, implement shared memory image extraction later
    const auto &raw_image = frame_msg.raw_image;
    if (raw_image.data.empty()) {
        RDX_INFO_DEV(this, __func__, false, "[f={}] Empty image data", __func__);
        return -1;
    }
    *output = cv_bridge::toCvCopy(raw_image)->image;
    return 0;
}

int Yolo8BodyPoseDetectorNode::_create_image_request_handler(const RuntimeConfig_t &runtime_config)
{
    using ProcessHandler_t = Impl::PullProcessSendHandler_t;
    using InputDataTrait_t = ByImageRequest::InputPort_t::ActionDataTrait_t;
    auto config = std::make_shared<ProcessHandler_t::InitConfig_t>();
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);

    config->block_input_reading = runtime_config.enable_blocking_mode;
    config->block_resource_acquisition = runtime_config.enable_blocking_mode;
    bool enable_visualization = runtime_config.enable_visualization;

    auto enqueue_policy = init_config->image_request_config->output_enqueue_policy;
    m_impl->work_then_send_handler = std::make_shared<ProcessHandler_t>();
    auto process_handler = m_impl->work_then_send_handler;
    process_handler->init(m_image_request_input_port.get(), m_image_request_output_port.get(),
                          &m_impl->inference_resource_pool, config, enqueue_policy);

    process_handler->on_process_input_data =
        [this, enable_visualization](auto *output_request, auto *action_result,
                                     auto source_data, auto &resource) {
            // extract image
            cv::Mat input_image;
            auto ret_extract_image = _extract_image(&input_image, source_data->get_goal()->frame);

            // do inference
            DetectionResult_t det_result;
            if (ret_extract_image == 0) {
                //! Log the resource index using RDX_INFO_DEV
                RDX_INFO_DEV(this, __func__, false, "Using inference resource index: {}", resource.index_in_pool);
                _do_inference(&det_result, input_image, resource);
            }

            // create output request
            ByImageRequest::OutputSourceData_t o_source;
            o_source.image = input_image;
            inference::conversion::to_ros_msg(&o_source.detections, det_result);

            ByImageRequest::OutputRequest_t request;
            request.set_source_data(o_source);
            request.set_control_signal_code(InputDataTrait_t::get_control_signal_code(*source_data->get_goal()));
            *output_request = request;

            // publish visualization
            if (m_impl->pub_visualization && enable_visualization && ret_extract_image == 0) {
                cv::Mat vis_canvas = input_image.clone();
                _draw_visualization(vis_canvas, det_result);
                m_impl->pub_visualization->publish(vis_canvas);
            }

            // FIXME: add this to config
            // publish detection done, used for timing measurement
            auto frame_num = source_data->get_goal()->frame.metadata.frame_num;
            RDX_INFO_DEV(this, __func__, false, "Publishing detection done, frame_num={}", frame_num);
            if (m_impl->pub_detection_done) {
                std_msgs::msg::String msg;
                msg.data = fmt::format("detection done,frame_num={}", frame_num);
                m_impl->pub_detection_done->publish(msg);
            }

            // fill the action result, nothing to do
            (void)action_result;
            return 0;
        };
    return 0;
}

int Yolo8BodyPoseDetectorNode::_do_inference(DetectionResult_t *output_result,
                                             const cv::Mat &input_image,
                                             const InferenceResource_t &resource,
                                             std::optional<UUIDType> msg_uuid)
{
    // FIXME: it looks like onnx inference has memory leak, need to investigate
    // do inference
    auto model = resource.model;
    auto inout_data = resource.inout_data;
    auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

    std::string msg_uuid_str;
    if (msg_uuid) {
        msg_uuid_str = UUIDTrait::to_string(*msg_uuid);
    }

    // inference
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Doing inference", msg_uuid_str);
    model->set_input_images(inout_data, {input_image}, RequiredImageEncoding);
    model->do_inference(inout_data);

    // get result
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Getting inference result", msg_uuid_str);
    auto output_detections = model->get_output_detections(inout_data,
                                                          config->model_output_config);

    // DO NOT return the resource here, although we do not need it anymore
    // if goal reply is slower than inference (although unlikely), we will have a lot of pending reply requests
    // m_impl->inference_resource_pool.push(inference_resource);
    if (output_result) {
        *output_result = output_detections[0];
    }

    return 0;
}

int Yolo8BodyPoseDetectorNode::_stop()
{
    _close_all_ports();
    return 0;
}

void Yolo8BodyPoseDetectorNode::_close_all_ports()
{
    // stop the input port from receiving new goals
    if (m_detection_request_input_port) {
        auto ret = m_detection_request_input_port->stop();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to stop detection request input port, error code: {}", __func__, ret);
        }
    }

    // stop the image request input port
    if (m_image_request_input_port) {
        auto ret = m_image_request_input_port->stop();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to stop image request input port, error code: {}", __func__, ret);
        }
    }

    // stop the image request output port
    if (m_image_request_output_port) {
        auto ret = m_image_request_output_port->stop();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to stop image request output port, error code: {}", __func__, ret);
        }
    }
}

// int Yolo8BodyPoseDetectorNode::_create_image_request_handler(const RuntimeConfig_t &runtime_config)
// {
//     (void)runtime_config;
//     return 0;
// }

int Yolo8BodyPoseDetectorNode::_create_detection_request_handler(const RuntimeConfig_t &runtime_config)
{
    using ActionDataTrait_t = ByDetectionRequest::InputPort_t::ActionDataTrait_t;

    auto handler_config = std::make_shared<Impl::PullProcessReplyHandler_t::InitConfig_t>();
    handler_config->block_input_reading = runtime_config.enable_blocking_mode;
    handler_config->block_resource_acquisition = runtime_config.enable_blocking_mode;
    bool enable_visualization = runtime_config.enable_visualization;

    m_impl->work_then_reply_handler = std::make_shared<Impl::PullProcessReplyHandler_t>();
    auto process_handler = m_impl->work_then_reply_handler;
    process_handler->init(
        m_detection_request_input_port.get(),
        &m_impl->inference_resource_pool,
        handler_config, this);

    Impl::PullProcessReplyHandler_t::OnProcessInputDataCallback_t process_func =
        [this, enable_visualization](Impl::PullProcessReplyHandler_t::InputActionResult_t *output_action_result,
                                     std::shared_ptr<Impl::PullProcessReplyHandler_t::InputSourceData_t> source_data,
                                     Impl::PullProcessReplyHandler_t::ResourceToken_t &resource_token) {
            auto msg_uuid = ActionDataTrait_t::get_uuid(*source_data->get_goal());
            std::string msg_uuid_str = UUIDTrait::to_string(msg_uuid);

            // Extract image from input data
            cv::Mat input_image;
            auto ret_extract_image = _extract_image(&input_image, source_data);
            if (ret_extract_image != 0) {
                RDX_RAISE_ERROR("[f={}] [msg_uid={}] Failed to extract image from input data, error code: {}",
                                __func__, msg_uuid_str, ret_extract_image);
            }

            if (input_image.empty()) {
                RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Empty image, aborting goal", msg_uuid_str);
                return -1;
            }

            // Perform inference
            DetectionResult_t det_result;
            auto ret_inference = _do_inference(&det_result, input_image, resource_token);
            if (ret_inference != 0) {
                RDX_RAISE_ERROR("[f={}] [msg_uid={}] Failed to do inference, error code: {}",
                                __func__, msg_uuid_str, ret_inference);
            }

            // Convert detections to ROS message
            inference::conversion::to_ros_msg(&output_action_result->detections, det_result);

            // Add frame metadata to each detection
            for (auto &detection : output_action_result->detections) {
                detection.frame_metadata = source_data->get_goal()->frame.metadata;
            }

            // publish visualization
            if (m_impl->pub_visualization && enable_visualization) {
                cv::Mat vis_canvas = input_image.clone();
                _draw_visualization(vis_canvas, det_result);
                m_impl->pub_visualization->publish(vis_canvas);
            }

            // Mark the goal as succeeded
            return 0;
        };
    process_handler->on_process_input_data = process_func;

    return 0;
}

int Yolo8BodyPoseDetectorNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Updating runtime config");
    (void)runtime_config;

    // initialize detection request handler
    auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(runtime_config);

    RDX_INFO_DEV(this, __func__, false, "{}", "Creating detection request handler");
    _create_detection_request_handler(*config);
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating image request handler");
    _create_image_request_handler(*config);

    RDX_INFO_DEV(this, __func__, false, "{}", "Updating runtime config completed");
    return 0;
}

int Yolo8BodyPoseDetectorNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> init_config)
{
    auto config = std::dynamic_pointer_cast<InitConfig_t>(init_config);
    if (!config) {
        RDX_RAISE_ERROR("[f={}] Failed to convert init config to Yolo8BodyPoseDetectorNode::InitConfig_t", __func__);
    }

    // NOTE: create all input ports first, and then output ports
    // otherwise you may get deadlocks because other ports are waiting for the input ports to be created

    // create input port and init it
    if (config->detection_request_config.has_value()) {
        m_detection_request_input_port = std::make_shared<ByDetectionRequest::InputPort_t>(this);
        {
            auto ret = m_detection_request_input_port->init(config->detection_request_config->input_port_config);
            if (ret != 0) {
                RDX_RAISE_ERROR("[f={}] Failed to init detection request input port, error code: {}", __func__, ret);
            }
        }
    }

    if (config->image_request_config.has_value()) {
        m_image_request_input_port = std::make_shared<ByImageRequest::InputPort_t>(this);
        {
            auto ret = m_image_request_input_port->init(config->image_request_config->input_port_config);
            if (ret != 0) {
                RDX_RAISE_ERROR("[f={}] Failed to init image request input port, error code: {}", __func__, ret);
            }
        }

        m_image_request_output_port = std::make_shared<ByImageRequest::OutputPort_t>(this);
        {
            auto ret = m_image_request_output_port->init(config->image_request_config->output_port_config);
            if (ret != 0) {
                RDX_RAISE_ERROR("[f={}] Failed to init image request output port, error code: {}", __func__, ret);
            }
        }
    }

    // create all inference resources
    {
        RDX_INFO_DEV(this, __func__, false, "{}", "Creating all inference resources");
        auto ret = _create_all_inference_resources(config->model_configs);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to create all inference resources, error code: {}", __func__, ret);
        }
    }

    // create visualization publisher
    if (!config->visualization_topic.empty()) {
        m_impl->pub_visualization = std::make_shared<StampedImagePub>(this, config->visualization_topic);
    }

    // create detection done publisher
    m_impl->pub_detection_done = this->create_publisher<std_msgs::msg::String>("probe/detection_done", 1000);

    return 0;
}

void Yolo8BodyPoseDetectorNode::_draw_visualization(cv::Mat &canvas,
                                                    const DetectionResult_t &detections)
{
    const auto &keypoint_connections = InferenceModel_t::get_keypoint_connections();
    for (const auto &obj : detections.objects) {
        //! Draw bounding box
        cv::rectangle(canvas,
                      cv::Point(obj.xywh[0], obj.xywh[1]),
                      cv::Point(obj.xywh[0] + obj.xywh[2], obj.xywh[1] + obj.xywh[3]),
                      cv::Scalar(0, 255, 0), 2);

        //! Draw keypoints
        for (size_t i = 0; i < obj.keypoints.size(); i++) {
            const auto &kp = obj.keypoints[i];
            if (kp.score > 0) { // Only draw if keypoint is detected
                cv::circle(canvas,
                           cv::Point(kp.xy[0], kp.xy[1]),
                           3, cv::Scalar(255, 0, 0), -1);
            }
        }

        //! Draw connections between keypoints to form skeleton
        for (const auto &connection : keypoint_connections) {
            const auto &kp1 = obj.keypoints[connection.first];
            const auto &kp2 = obj.keypoints[connection.second];
            if (kp1.score > 0 && kp2.score > 0) {
                cv::line(canvas,
                         cv::Point(kp1.xy[0], kp1.xy[1]),
                         cv::Point(kp2.xy[0], kp2.xy[1]),
                         cv::Scalar(0, 0, 255), 2);
            }
        }
    }
}

int Yolo8BodyPoseDetectorNode::_process_detection_request()
{
    auto ret = m_impl->work_then_reply_handler->process_and_reply();
    if (ret == Impl::PullProcessReplyHandler_t::ProcessResult::Error) {
        RDX_INFO_DEV(this, __func__, false, "Failed to process detection request, error code: {}", int(ret));
        return -1;
    } else if (ret == Impl::PullProcessReplyHandler_t::ProcessResult::NoData) {
        // RDX_INFO_DEV(this, __func__, false, "{}", "No data available, skipping");
        return 0;
    } else if (ret == Impl::PullProcessReplyHandler_t::ProcessResult::Success) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Successfully processed detection request");
        return 0;
    } else if (ret == Impl::PullProcessReplyHandler_t::ProcessResult::NoResourceToken) {
        RDX_INFO_DEV(this, __func__, false, "{}", "No resource token, skipping");
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

int Yolo8BodyPoseDetectorNode::_process_image_request()
{
    auto ret = m_impl->work_then_send_handler->process_and_send();
    if (ret == Impl::PullProcessSendHandler_t::ProcessResult::Error) {
        RDX_INFO_DEV(this, __func__, false, "Failed to process image request, error code: {}", int(ret));
        return -1;
    } else if (ret == Impl::PullProcessSendHandler_t::ProcessResult::NoData) {
        //! No data available, skipping
        return 0;
    } else if (ret == Impl::PullProcessSendHandler_t::ProcessResult::Success) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Successfully processed image request");
        return 0;
    } else if (ret == Impl::PullProcessSendHandler_t::ProcessResult::NoResourceToken) {
        //! No resource token, skipping
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

void Yolo8BodyPoseDetectorNode::_step()
{
    if (m_detection_request_input_port) {
        _process_detection_request();
    }
    if (m_image_request_input_port) {
        _process_image_request();
    }
}

int Yolo8BodyPoseDetectorNode::_create_inference_resource(
    InitConfig_t::ModelConfig_t::Ptr model_config,
    int replicas)
{
    // create a new inference resource, and push it to the concurrent queue
    // @return 0 if success, -1 if failed
    // load model
    auto model = std::make_shared<inference::yolo8::Yolo8PoseModel>();
    auto model_init_params = model->create_init_params();
    {
        auto _init_params = std::dynamic_pointer_cast<inference::yolo8::Yolo8ModelConfig>(model_init_params);
        if (!_init_params) {
            RDX_RAISE_ERROR("Failed to cast model init params to Yolo8ModelConfig");
        }
        *_init_params = *model_config;
    }
    auto ret_model_open = model->open(model_init_params);
    if (ret_model_open != 0) {
        RDX_RAISE_ERROR("Failed to open model, error code: {}", ret_model_open);
        return -1;
    }

    for (int i = 0; i < replicas; ++i) {
        // create inference inout data
        auto inference_inout_data = model->create_inference_inout_data();

        // create inference resource
        InferenceResource_t inference_resource;
        inference_resource.model = model;
        inference_resource.inout_data = inference_inout_data;
        inference_resource.model_config = model_config;
        inference_resource.replica_id = i;
        inference_resource.index_in_pool = m_impl->inference_resource_pool.size();
        m_impl->inference_resource_pool.push(inference_resource);
    }
    return 0;
}

int Yolo8BodyPoseDetectorNode::_create_all_inference_resources(
    const std::vector<InitConfig_t::ModelConfig_t::Ptr> &model_configs)
{
    if (model_configs.empty()) {
        RDX_INFO_DEV(this, __func__, false, "{}", "No model configs, skipping model resource creation");
        return 0;
    }

    // setup capacity of the inference resource pool
    m_impl->inference_resource_pool.set_capacity(model_configs.size());

    // now fill the pool with inference resources
    std::map<InitConfig_t::ModelConfig_t::Ptr, int> model_config_to_replicas;

    // count the number of replicas for each model config
    for (const auto &model_config : model_configs) {
        model_config_to_replicas[model_config] += 1;
    }

    // create inference resources for each model config
    for (const auto &[model_config, replicas] : model_config_to_replicas) {
        auto ret = _create_inference_resource(model_config, replicas);
        if (ret != 0) {
            RDX_RAISE_ERROR("Failed to create inference resource, error code: {}", ret);
            return ret;
        }
    }

    RDX_INFO_DEV(this, __func__, false, "Successfully created all inference resources, total = {}", m_impl->inference_resource_pool.size());
    return 0;
}
} // namespace redoxi_works::model_nodes
