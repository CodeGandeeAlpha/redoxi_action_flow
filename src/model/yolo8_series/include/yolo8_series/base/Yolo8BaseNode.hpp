#pragma once

#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <yolo8_series/base/Yolo8BaseTypes.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>
#include <redoxi_dnn_models/message_conversion.hpp>
#include <redoxi_dnn_models/visualizations.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessReplyHandler.hpp>


namespace redoxi_works::model_nodes::yolo8
{

template <YoloModelConcept TModel>
class Yolo8BaseNode : public redoxi_works::common_nodes::StartStopNode
{
  public:
    inline static constexpr const char *RequiredImageEncoding = sensor_msgs::image_encodings::RGB8;

  public:
    using InferenceModel_t = TModel;
    struct ByDetectionRequest {
        using InputPort_t = DetectionRequestInputPort;
        using InputAction_t = typename InputPort_t::ActionType_t;
        using InputActionDataTrait_t = typename InputPort_t::ActionDataTrait_t;
        using InputGoalUUID_t = typename InputPort_t::GoalUUID_t;
        using InputSourceData_t = typename InputPort_t::SourceData_t;
        using InputActionResult_t = typename InputPort_t::ActionResult_t;
    };

    struct ByImageRequest {
        using InputPort_t = ImageRequestInputPort;
        using InputAction_t = typename InputPort_t::ActionType_t;
        using InputActionDataTrait_t = typename InputPort_t::ActionDataTrait_t;
        using InputGoalUUID_t = typename InputPort_t::GoalUUID_t;
        using InputSourceData_t = typename InputPort_t::SourceData_t;

        using OutputPort_t = ImageRequestOutputPort;
        using OutputAction_t = typename OutputPort_t::ActionType_t;
        using OutputActionDataTrait_t = typename OutputPort_t::ActionDataTrait_t;
        using OutputSourceData_t = typename OutputPort_t::SourceData_t;
        using OutputRequest_t = typename OutputPort_t::DeliveryRequest_t;
    };

    using InitConfig_t = InitConfig<InferenceModel_t>;
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using RuntimeConfig_t = RuntimeConfig<InferenceModel_t>;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;
    using InferenceResource_t = InferenceResource<InferenceModel_t>;
    using DetectionResult_t = typename InferenceModel_t::SingleImageOutput_t;

  public:
    explicit Yolo8BaseNode(const std::string &node_name,
                           const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~Yolo8BaseNode() noexcept;

  protected:
    // from base class
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

  protected:
    // from this class
    virtual int _extract_image(cv::Mat *output, const std::shared_ptr<typename ByDetectionRequest::InputSourceData_t> &source_data);

    virtual int _process_detection_request();

    //! create a new pull process reply handler for detection request
    virtual int _create_detection_request_handler(const RuntimeConfig_t &runtime_config);

    //! in image request, convert the detection result to ROS message, for sending into output port
    virtual int _process_detection_result(typename ByImageRequest::OutputSourceData_t *output_source_data,
                                          const DetectionResult_t &det_result,
                                          const typename ByImageRequest::InputSourceData_t &source_data,
                                          const cv::Mat &input_image);

    //! extract image from source data
    virtual int _extract_image(cv::Mat *output, const std::shared_ptr<typename ByImageRequest::InputSourceData_t> &source_data);

    //! create a new pull process send handler for image request
    virtual int _create_image_request_handler(const RuntimeConfig_t &runtime_config);

    //! in detection request, convert the detection result to ROS message, for replying to client
    virtual int _process_detection_result(typename ByDetectionRequest::InputActionResult_t *output_action_result,
                                          const DetectionResult_t &det_result,
                                          const typename ByDetectionRequest::InputSourceData_t &source_data,
                                          const cv::Mat &input_image);

    //! process image request
    virtual int _process_image_request();

    //! Draw visualization on canvas
    virtual void _draw_visualization(cv::Mat &canvas,
                                     const DetectionResult_t &detections);

    //! create a new inference resource, and push it to the concurrent queue
    //! @param replicas: number of replicas to create, replicated resource will share the same model but with different inout data
    //! @return 0 if success, -1 if failed
    virtual int _create_inference_resource(
        std::shared_ptr<typename InitConfig_t::ModelConfig_t> model_config,
        int replicas = 1);

    //! create all inference resources
    virtual int _create_all_inference_resources(
        const std::vector<std::shared_ptr<typename InitConfig_t::ModelConfig_t>> &model_configs);


  protected:
    std::shared_ptr<typename ByDetectionRequest::InputPort_t> m_detection_request_input_port;
    std::shared_ptr<typename ByImageRequest::InputPort_t> m_image_request_input_port;
    std::shared_ptr<typename ByImageRequest::OutputPort_t> m_image_request_output_port;

  private:
    struct Impl {
        using Node_t = Yolo8BaseNode;
        Impl() = default;

        // this limits the number of inference resources, like GPU, NPU
        // each task must first acquire a resource from this pool, then do inference
        tbb::concurrent_bounded_queue<InferenceResource_t> inference_resource_pool;

        // visualization publisher
        std::shared_ptr<StampedImagePub> pub_visualization;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_detection_done;

        // pull input, work on it and then send output
        using PullProcessSendHandler_t = redoxi_works::port_handlers::PullProcessSendHandler<typename Node_t::ByImageRequest::InputPort_t::MasterSpec_t,
                                                                                             typename Node_t::ByImageRequest::OutputPort_t::MasterSpec_t,
                                                                                             InferenceResource_t>;
        std::shared_ptr<PullProcessSendHandler_t> work_then_send_handler;

        // pull input, work on it and then reply
        using PullProcessReplyHandler_t = redoxi_works::port_handlers::PullProcessReplyHandler<typename Node_t::ByDetectionRequest::InputPort_t::MasterSpec_t,
                                                                                               InferenceResource_t>;
        std::shared_ptr<PullProcessReplyHandler_t> work_then_reply_handler;
    };

    std::shared_ptr<Impl> m_impl;

    int _extract_image(cv::Mat *output, const redoxi_public_msgs::msg::Frame &frame_msg);
    int _do_inference(DetectionResult_t *output_result,
                      const cv::Mat &input_image,
                      const InferenceResource_t &resource,
                      std::optional<UUIDType> msg_uuid = std::nullopt);
    void _close_all_ports();
};

template <YoloModelConcept TModel>
Yolo8BaseNode<TModel>::Yolo8BaseNode(const std::string &node_name,
                                     const rclcpp::NodeOptions &options)
    : redoxi_works::common_nodes::StartStopNode(node_name, options)
{
    m_impl = std::make_shared<Impl>();
}


template <YoloModelConcept TModel>
Yolo8BaseNode<TModel>::~Yolo8BaseNode() noexcept
{
    _close_all_ports();
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_start()
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

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_extract_image(cv::Mat *output,
                                          const std::shared_ptr<typename ByDetectionRequest::InputSourceData_t> &source_data)
{
    if (!output) {
        //! nothing to do
        return 0;
    }

    return _extract_image(output, source_data->get_goal()->frame_bundle.primary_frame);
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_extract_image(cv::Mat *output,
                                          const std::shared_ptr<typename ByImageRequest::InputSourceData_t> &source_data)
{
    if (!output) {
        //! nothing to do
        return 0;
    }

    return _extract_image(output, source_data->get_goal()->frame_bundle.primary_frame);
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_extract_image(cv::Mat *output,
                                          const redoxi_public_msgs::msg::Frame &frame_msg)
{
    if (!output) {
        //! nothing to do
        return 0;
    }

    //! TODO: currently only support raw image, implement shared memory image extraction later
    image_utils::FrameMediator fm(&frame_msg);
    fm.to_cv_image_copy(*output);
    return 0;
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_process_detection_result(typename ByImageRequest::OutputSourceData_t *output_source_data,
                                                     const DetectionResult_t &det_result,
                                                     const typename ByImageRequest::InputSourceData_t &source_data,
                                                     const cv::Mat &input_image)
{
    (void)input_image;

    // convert detection result to ROS message
    inference::conversion::to_ros_msg(&output_source_data->detections, det_result);

    // add frame metadata to each detection
    for (auto &detection : output_source_data->detections) {
        detection.frame_metadata = source_data.get_goal()->frame_bundle.primary_frame.metadata;
    }

    ImageRequestOutputPort::SourceData_t::FrameData_t frame_data;
    frame_data.from_frame_msg(source_data.get_goal()->frame_bundle.primary_frame);
    output_source_data->frame_data = frame_data;

    // set the image
    // output_source_data->image = input_image;
    // image_ports::types::DeliverySourceData _output_source_data;
    // image_ports::types::DeliverySourceData::FrameData_t frame_data;
    // output_source_data->from_frame_bundle(source_data.get_goal()->frame_bundle);
    return 0;
}

//! in detection request, convert the detection result to ROS message, for replying to client
template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_process_detection_result(typename ByDetectionRequest::InputActionResult_t *output_action_result,
                                                     const DetectionResult_t &det_result,
                                                     const typename ByDetectionRequest::InputSourceData_t &source_data,
                                                     const cv::Mat &input_image)
{
    // convert detection result to ROS message
    (void)input_image;
    (void)source_data;
    inference::conversion::to_ros_msg(&output_action_result->detections, det_result);

    // add frame metadata to each detection
    image_utils::FrameMediator fm(&source_data.get_goal()->frame_bundle.primary_frame);
    for (auto &detection : output_action_result->detections) {
        detection.frame_metadata = fm.get_metadata();
    }

    return 0;
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_create_image_request_handler(const RuntimeConfig_t &runtime_config)
{
    // TODO: handle the case where output port is nullptr but input port is valid
    // should still work
    if (!m_image_request_input_port || !m_image_request_output_port) {
        RDX_INFO_DEV(this, __func__, false, "[f={}] No image request input or output port, skip", __func__);
        return 0;
    }

    using ProcessHandler_t = typename Impl::PullProcessSendHandler_t;
    using InputDataTrait_t = typename ByImageRequest::InputPort_t::ActionDataTrait_t;
    auto config = std::make_shared<typename ProcessHandler_t::InitConfig_t>();
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);

    config->block_input_reading = runtime_config.enable_blocking_mode;
    config->block_resource_acquisition = runtime_config.enable_blocking_mode;
    bool enable_visualization = runtime_config.enable_visualization;
    bool enable_performance_probe = runtime_config.enable_performance_probe;
    auto enqueue_policy = init_config->image_request_config->output_enqueue_policy;
    m_impl->work_then_send_handler = std::make_shared<ProcessHandler_t>();
    auto process_handler = m_impl->work_then_send_handler;
    process_handler->init(m_image_request_input_port.get(), m_image_request_output_port.get(),
                          &m_impl->inference_resource_pool, config, enqueue_policy);

    process_handler->on_process_input_data =
        [this,
         enable_visualization,
         enable_performance_probe](typename ProcessHandler_t::OutputRequest_t *output_request,
                                   std::optional<typename ProcessHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                   typename ProcessHandler_t::InputActionResult_t *action_result,
                                   std::shared_ptr<const typename ByImageRequest::InputSourceData_t> source_data,
                                   typename ProcessHandler_t::ResourceToken_t &resource) {
            (void)output_enqueue_policy;
            // extract image
            cv::Mat input_image;
            image_utils::FrameMediator fm(&source_data->get_goal()->frame_bundle.primary_frame);
            auto ret_extract_image = fm.to_cv_image_copy(input_image);
            // auto ret_extract_image = _extract_image(&input_image, source_data->get_goal()->frame);

            // do inference
            DetectionResult_t det_result;
            if (ret_extract_image == 0) {
                //! Log the resource index using RDX_INFO_DEV
                RDX_INFO_DEV(this, __func__, false, "Using inference resource index: {}", resource.index_in_pool);
                _do_inference(&det_result, input_image, resource);
            }

            // create output request
            typename ByImageRequest::OutputSourceData_t o_source;
            // o_source.image = input_image;
            // inference::conversion::to_ros_msg(&o_source.detections, det_result);
            _process_detection_result(&o_source, det_result, *source_data, input_image);

            typename ByImageRequest::OutputRequest_t request;
            request.set_source_data(o_source);
            auto signal_code = InputDataTrait_t::get_control_signal_code(*source_data->get_goal());
            request.set_control_signal_code(signal_code);
            *output_request = request;

            // special control signal must be delivered reliably
            // if (signal_code != ControlSignalCode::Normal && signal_code != ControlSignalCode::Ping) {
            //     auto qos = (*output_enqueue_policy).value_or(typename ProcessHandler_t::OutputDeliveryPolicy_t());
            //     qos.set_precondition(DeliveryPrecondition::NoPrecondition);
            //     qos.set_drop_strategy(DropStrategy::NoDrop);
            //     *output_enqueue_policy = qos;
            // }

            // publish visualization
            if (m_impl->pub_visualization && enable_visualization && ret_extract_image == 0) {
                cv::Mat vis_canvas = input_image.clone();
                _draw_visualization(vis_canvas, det_result);
                RDX_INFO_DEV(this, __func__, false, "Publishing visualization with encoding: {}", fm.get_encoding());
                // {
                //     // directly show it
                //     cv::imshow("vis_canvas", vis_canvas);
                //     cv::waitKey(0);
                // }
                m_impl->pub_visualization->publish(vis_canvas, fm.get_encoding());
            }

            //! Print probe message
            if (enable_performance_probe && m_impl->pub_detection_done) {
                std_msgs::msg::String msg;
                msg.data = fmt::format("detection request done, frame_num={}",
                                       source_data->get_goal()->frame_bundle.primary_frame.metadata.frame_num);
                m_impl->pub_detection_done->publish(msg);
            }

            // fill the action result, nothing to do
            (void)action_result;
            return 0;
        };
    return 0;
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_create_detection_request_handler(const RuntimeConfig_t &runtime_config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating detection request handler");
    using ActionDataTrait_t = typename ByDetectionRequest::InputPort_t::ActionDataTrait_t;

    if (!m_detection_request_input_port) {
        RDX_INFO_DEV(this, __func__, false, "[f={}] No detection request input port, skip", __func__);
        return 0;
    }

    auto handler_config = std::make_shared<typename Impl::PullProcessReplyHandler_t::InitConfig_t>();
    handler_config->block_input_reading = runtime_config.enable_blocking_mode;
    handler_config->block_resource_acquisition = runtime_config.enable_blocking_mode;
    bool enable_visualization = runtime_config.enable_visualization;
    bool enable_performance_probe = runtime_config.enable_performance_probe;
    m_impl->work_then_reply_handler = std::make_shared<typename Impl::PullProcessReplyHandler_t>();
    auto process_handler = m_impl->work_then_reply_handler;

    RDX_INFO_DEV(this, __func__, false, "Initializing detection request handler, action name: {}",
                 m_detection_request_input_port->get_config()->get_action_name());

    process_handler->init(
        m_detection_request_input_port.get(),
        &m_impl->inference_resource_pool,
        handler_config);

    typename Impl::PullProcessReplyHandler_t::OnProcessInputDataCallback_t process_func =
        [this, enable_visualization, enable_performance_probe](typename Impl::PullProcessReplyHandler_t::InputActionResult_t *output_action_result,
                                                               std::shared_ptr<typename Impl::PullProcessReplyHandler_t::InputSourceData_t> source_data,
                                                               typename Impl::PullProcessReplyHandler_t::ResourceToken_t &resource_token) {
            auto msg_uuid = ActionDataTrait_t::get_uuid(*source_data->get_goal());
            std::string msg_uuid_str = UUIDTrait::to_string(msg_uuid);
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Got detection request", msg_uuid_str);

            // Extract image from input data
            cv::Mat input_image;

            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Extracting image from input data", msg_uuid_str);
            image_utils::FrameMediator fm(&source_data->get_goal()->frame_bundle.primary_frame);
            auto ret_extract_image = fm.to_cv_image_copy(input_image);
            if (ret_extract_image != 0) {
                RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Failed to extract image from input data, error code: {}",
                             msg_uuid_str, ret_extract_image);
                return -1;
            }

            if (input_image.empty()) {
                RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Extracted empty image, no detection will be done", msg_uuid_str);
                return 0;
            }

            // Perform inference
            DetectionResult_t det_result;
            auto ret_inference = _do_inference(&det_result, input_image, resource_token);
            if (ret_inference != 0) {
                RDX_RAISE_ERROR("[f={}] [msg_uid={}] Failed to do inference, error code: {}",
                                __func__, msg_uuid_str, ret_inference);
            }

            // Convert detections to ROS message
            _process_detection_result(output_action_result, det_result, *source_data, input_image);

            // publish visualization
            if (m_impl->pub_visualization && enable_visualization) {
                cv::Mat vis_canvas = input_image.clone();
                _draw_visualization(vis_canvas, det_result);
                RDX_INFO_DEV(this, __func__, false, "Publishing visualization with encoding: {}", fm.get_encoding());
                // {
                //     // directly show it
                //     cv::imshow("vis_canvas", vis_canvas);
                //     cv::waitKey(0);
                // }
                m_impl->pub_visualization->publish(vis_canvas, fm.get_encoding());
            }

            //! Print probe message
            if (enable_performance_probe && m_impl->pub_detection_done) {
                std_msgs::msg::String msg;
                msg.data = fmt::format("detection request done, frame_num={}",
                                       source_data->get_goal()->frame_bundle.primary_frame.metadata.frame_num);
                m_impl->pub_detection_done->publish(msg);
            }

            // Mark the goal as succeeded
            return 0;
        };
    process_handler->on_process_input_data = process_func;

    RDX_INFO_DEV(this, __func__, false, "{}", "Detection request handler completed");

    return 0;
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_stop()
{
    _close_all_ports();
    return 0;
}

template <YoloModelConcept TModel>
void Yolo8BaseNode<TModel>::_close_all_ports()
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

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_do_inference(DetectionResult_t *output_result,
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

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
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

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_update_init_config(std::shared_ptr<BaseInitConfig_t> init_config)
{
    auto config = std::dynamic_pointer_cast<InitConfig_t>(init_config);
    if (!config) {
        RDX_RAISE_ERROR("[f={}] Failed to convert init config to Yolo8BodyPoseDetectorNode::InitConfig_t", __func__);
    }

    // NOTE: create all input ports first, and then output ports
    // otherwise you may get deadlocks because other ports are waiting for the input ports to be created

    // create input port and init it
    if (config->detection_request_config.has_value()) {
        m_detection_request_input_port = std::make_shared<typename ByDetectionRequest::InputPort_t>(this);
        {
            auto ret = m_detection_request_input_port->init(config->detection_request_config->input_port_config);
            if (ret != 0) {
                RDX_RAISE_ERROR("[f={}] Failed to init detection request input port, error code: {}", __func__, ret);
            }
        }
    }

    if (config->image_request_config.has_value()) {
        m_image_request_input_port = std::make_shared<typename ByImageRequest::InputPort_t>(this);
        {
            auto ret = m_image_request_input_port->init(config->image_request_config->input_port_config);
            if (ret != 0) {
                RDX_RAISE_ERROR("[f={}] Failed to init image request input port, error code: {}", __func__, ret);
            }
        }

        m_image_request_output_port = std::make_shared<typename ByImageRequest::OutputPort_t>(this);
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
    if (!config->publish_visualization_topic.empty()) {
        m_impl->pub_visualization = std::make_shared<StampedImagePub>(this, config->publish_visualization_topic);
    }

    // create detection done publisher
    if (!config->publish_probe_detection_done_topic.empty()) {
        m_impl->pub_detection_done = this->create_publisher<std_msgs::msg::String>(
            config->publish_probe_detection_done_topic,
            DefaultParams::ProbePublisherQoS);
    }

    return 0;
}

template <YoloModelConcept TModel>
void Yolo8BaseNode<TModel>::_draw_visualization(cv::Mat &canvas,
                                                const DetectionResult_t &detections)
{
    using namespace redoxi_works::dnn_models::visualizations;
    DrawDetectionsOptions options;
    options.colorization_mode = DrawDetectionsOptions::ColorizationMode::ClassId;
    draw_detections(&canvas, detections, options);
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_process_detection_request()
{
    if (!m_impl->work_then_reply_handler) {
        // no detection request handler, error, should not reach here
        RDX_RAISE_ERROR("[f={}] No detection request handler, should not reach here", __func__);
    }

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

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_process_image_request()
{
    if (!m_impl->work_then_send_handler) {
        // no image request handler, error, should not reach here
        RDX_RAISE_ERROR("[f={}] No image request handler, should not reach here", __func__);
    }

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
    } else if (ret == Impl::PullProcessSendHandler_t::ProcessResult::FailedToSend) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to send image request to downstream, do you have a downstream?");
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

template <YoloModelConcept TModel>
void Yolo8BaseNode<TModel>::_step()
{
    // RDX_INFO_DEV(this, __func__, false, "{}", "Stepping");
    if (m_detection_request_input_port && m_impl->work_then_reply_handler) {
        _process_detection_request();
    }
    if (m_image_request_input_port && m_impl->work_then_send_handler) {
        _process_image_request();
    }
}

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_create_inference_resource(
    std::shared_ptr<typename InitConfig_t::ModelConfig_t> model_config,
    int replicas)
{
    // create a new inference resource, and push it to the concurrent queue
    // @return 0 if success, -1 if failed
    // load model
    auto model = std::make_shared<InferenceModel_t>();
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

template <YoloModelConcept TModel>
int Yolo8BaseNode<TModel>::_create_all_inference_resources(
    const std::vector<std::shared_ptr<typename InitConfig_t::ModelConfig_t>> &model_configs)
{
    if (model_configs.empty()) {
        RDX_INFO_DEV(this, __func__, false, "{}", "No model configs, skipping model resource creation");
        return 0;
    }

    // setup capacity of the inference resource pool
    m_impl->inference_resource_pool.set_capacity(model_configs.size());

    // now fill the pool with inference resources
    std::map<std::shared_ptr<typename InitConfig_t::ModelConfig_t>, int> model_config_to_replicas;

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


} // namespace redoxi_works::model_nodes::yolo8
