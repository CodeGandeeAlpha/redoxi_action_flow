#include <psg_master_node/MasterNodeTypes.hpp>
#include <psg_master_node/MasterNode.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_samples_lib/random_image.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <json_struct/json_struct.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{

struct PSGMasterNodeImpl {
    using Node_t = PSGMasterNode;
    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    // pull input, work on it and then send output
    using PullProcessSendHandler_t = redoxi_works::port_handlers::PullProcessSendHandler<Node_t::InputPort_t::MasterSpec_t,
                                                                                         Node_t::OutputPort_t::MasterSpec_t>;
    std::shared_ptr<PullProcessSendHandler_t> work_then_send_handler;
};

PSGMasterNode::PSGMasterNode(const std::string &name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(name, options)
{
}

PSGMasterNode::~PSGMasterNode()
{
    // wait for all requests to be processed
    if (m_primary_output_port) {
        m_primary_output_port->wait_for_all_requests();
    }

    // stop ros time token
    if (m_impl->m_ros_time_token) {
        m_impl->m_ros_time_token->stop();
    }
}

int PSGMasterNode::_start()
{
    auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Start input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting psg master node");
    m_input_port->start();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port started");

    //! Start primary output port
    if (m_primary_output_port) {
        auto ret = m_primary_output_port->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] Failed to start primary output port, ret={}", __func__, ret);
            return ret;
        }
    }

    //! start ros time token
    {
        auto interval = config->frame_interval;
        m_impl->m_ros_time_token->start(interval);
    }

    return 0;
}

int PSGMasterNode::_stop()
{
    //! Stop input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Stopping psg master node");
    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");


    //! Stop primary output port
    if (m_primary_output_port) {
        m_primary_output_port->stop();
    }

    //! stop ros time token
    m_impl->m_ros_time_token->stop();

    return 0;
}

void PSGMasterNode::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
    if (m_publish_to_debug_topic) {
        if (m_primary_output_port) {
            m_primary_output_port->set_publish_to_debug_topic(enable);
        }
    }
}

bool PSGMasterNode::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGMasterNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{

    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);

    // parse the config into a string and print it
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "parse init config into a string");
    auto config_str = JS::serializeStruct(*init_config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "init config: {}", config_str);

    // create impl
    m_impl = _create_impl();

    //! Initialize output ports
    auto primary_output_port = _create_primary_output_port(*init_config);
    if (!primary_output_port) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port = primary_output_port;

    //! Create and initialize input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port");
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    //! Initialize debug publishers
    if (init_config->create_debug_pub) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "initialize debug publishers, enqueue topic={}, drop topic={}",
                     init_config->debug_pub_task_enqueue_name,
                     init_config->debug_pub_task_drop_name);
        auto debug_qos = DefaultParams::DebugPublisherQoS;
        m_pub_task_enqueue.init(this, init_config->debug_pub_task_enqueue_name, debug_qos);
        m_pub_task_drop.init(this, init_config->debug_pub_task_drop_name, debug_qos);
    }

    return 0;
}

int PSGMasterNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);

    //! parse the config into a string and print it
    auto config_str = JS::serializeStruct(*runtime_config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "runtime config: {}", config_str);

    //! set callback on request enqueued to resize image if needed
    m_primary_output_port->set_callback_on_request_enqueued([](DeliveryRequest_t &request) {
        // do nothing
    });

    //! set publish to debug topic
    set_publish_to_debug_topic(runtime_config->publish_to_debug_topic);

    RDX_INFO_DEV(this, __func__, false, "{}", "Creating document request handler");
    _create_document_request_handler(*runtime_config);

    return 0;
}

std::shared_ptr<PSGMasterNodeImpl> PSGMasterNode::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGMasterNodeImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

PSGMasterNode::DeliveryRequest_t
    PSGMasterNode::_create_delivery_request(const OutputSourceData_t &source_data,
                                            std::optional<ControlSignalCode> control_signal_code)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Create delivery request
    DeliveryRequest_t req;
    req.set_source_data(source_data);
    if (runtime_config->frame_request_policy.has_value()) {
        req.set_delivery_policy(*runtime_config->frame_request_policy);
    }
    if (control_signal_code.has_value()) {
        req.set_control_signal_code(control_signal_code.value());
    }

    return req;
}

std::shared_ptr<PSGMasterNode::OutputPort_t>
    PSGMasterNode::_create_primary_output_port(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port");
    auto port = std::make_shared<OutputPort_t>(this);
    auto &port_config = init_config.output_port_config;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);
    port->init(port_config);

    // // register callbacks
    // port->set_callback_on_deliver_task_begin([this](TargetData_t &target_data, const DeliveryTask_t &task) {
    //     return _on_delivery_task_begin(target_data, task.get_request());
    // });
    // port->set_callback_on_deliver_task_finish([this](TargetData_t &target_data, const DeliveryTask_t &task, const DeliveryResult_t &result) {
    //     return _on_delivery_task_finish(target_data, task.get_request(), result);
    // });
    // port->set_callback_on_deliver_to_downstream_finish([this](TargetData_t &target_data, SendResult_t &result, const Downstream_t &ds) {
    //     return _on_deliver_to_downstream_finish(target_data, result, ds);
    // });

    return port;
}

int PSGMasterNode::_create_document_request_handler(const RuntimeConfig_t &runtime_config)
{
    using ProcessHandler_t = PSGMasterNodeImpl::PullProcessSendHandler_t;
    using InputDataTrait_t = PSGMasterNode::InputPort_t::ActionDataTrait_t;
    auto config = std::make_shared<ProcessHandler_t::InitConfig_t>();
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);

    config->block_input_reading = runtime_config.enable_blocking_mode;
    config->block_resource_acquisition = runtime_config.enable_blocking_mode;

    auto enqueue_policy = runtime_config.frame_enqueue_policy;
    m_impl->work_then_send_handler = std::make_shared<ProcessHandler_t>();
    auto process_handler = m_impl->work_then_send_handler;
    process_handler->init(m_input_port.get(), m_primary_output_port.get(),
                          nullptr, config, enqueue_policy);

    process_handler->on_process_input_data =
        [this](ProcessHandler_t::OutputRequest_t *output_request,
               std::optional<ProcessHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
               ProcessHandler_t::InputActionResult_t *action_result,
               std::shared_ptr<const InputSourceData_t> source_data,
               ProcessHandler_t::ResourceToken_t &resource) {
            // from input source data to output source data
            OutputSourceData_t output_source_data;
            OutputSourceData_t::PSGDocument_t msg;
            msg.frame_bundle = source_data->m_goal->frame_bundle;

            auto goal_handle = source_data->get_goal_handle_future().get();
            auto control_signal_code = InputDataTrait_t::get_control_signal_code(*source_data->get_goal());
            RDX_INFO_DEV(this, __func__, true,
                         "on_process_input_data()中frame num: {}, control signal code: {}",
                         msg.frame_bundle.primary_frame.metadata.frame_num, int(control_signal_code));
            msg.frame_bundle.x_control.code = static_cast<int32_t>(control_signal_code);
            msg.frame_bundle.primary_frame.x_control.code = static_cast<int32_t>(control_signal_code);
            msg.x_control.code = static_cast<int32_t>(control_signal_code);
            auto task_metadata = ActionDataTrait_t::get_source_task_metadata(*source_data->get_goal());
            msg.x_task_metadata.source_task_info = task_metadata.source_task_info;
            msg.x_task_metadata.source_task_uid = to_ros_uuid_msg(task_metadata.source_task_id);
            output_source_data.set_document(msg);


            // create delivery request
            auto delivery_request = _create_delivery_request(output_source_data, control_signal_code);
            *output_request = delivery_request;

            RDX_INFO_DEV(this, __func__, true,
                         "output_request signal code: {}",
                         control_signal_code_to_string(output_request->get_control_signal_code()));

            image_utils::FrameMediator fm(&msg.frame_bundle.primary_frame);
            sensor_msgs::msg::Image image_msg;
            fm.to_image_msg(image_msg);
            m_pub_task_enqueue.publish(image_msg);

            // fill the action result, nothing to do
            (void)action_result;

            (void)output_enqueue_policy;
            (void)resource;
            return 0;
        };
    return 0;
}

int PSGMasterNode::_process_document_request()
{
    auto ret = m_impl->work_then_send_handler->process_and_send();
    if (ret == PSGMasterNodeImpl::PullProcessSendHandler_t::ProcessResult::Error) {
        RDX_INFO_DEV(this, __func__, false, "Failed to process image request, error code: {}", int(ret));
        return -1;
    } else if (ret == PSGMasterNodeImpl::PullProcessSendHandler_t::ProcessResult::NoData) {
        //! No data available, skipping
        return 0;
    } else if (ret == PSGMasterNodeImpl::PullProcessSendHandler_t::ProcessResult::Success) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Successfully processed image request");
        return 0;
    } else if (ret == PSGMasterNodeImpl::PullProcessSendHandler_t::ProcessResult::NoResourceToken) {
        //! No resource token, skipping
        return 0;
    } else if (ret == PSGMasterNodeImpl::PullProcessSendHandler_t::ProcessResult::FailedToSend) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to send image request to downstream, do you have a downstream?");
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

void PSGMasterNode::_step()
{
    if (m_input_port) {
        _process_document_request();
    }
}

// void PSGMasterNode::_step2()
// {
//     auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
//     if (get_status() != NodeStatusCode::STARTED) {
//         return;
//     }

//     RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "step once");

//     //! 随机构造input source data
//     auto frame_data = std::make_shared<InputSourceData_t>();
//     auto goal = std::make_shared<InputSourceData_t::ActionType_t::Goal>();
//     goal->frame = redoxi_public_msgs::msg::Frame();

//     //! 修改这部分代码以确保正确的图像格式
//     cv::Mat img;
//     random_image_with_shapes(img, cv::Size(640, 480));

//     //! 确保图像转换为正确的格式
//     auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
//     goal->frame.raw_image = *msg;

//     //! 设置其他必要的图像信息
//     goal->frame.raw_image.header.stamp = this->now();
//     goal->frame.raw_image.header.frame_id = "camera";

//     frame_data->set_goal(goal);

//     RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "after set goal");
//     // frame_data->m_goal = std::make_shared<InputSourceData_t::Goal>();
//     // frame_data->m_goal->frame = cv::Mat(480, 640, CV_8UC3, cv::Scalar(rand()%255, rand()%255, rand()%255));

//     // from input source data to output source data
//     OutputSourceData_t output_source_data;
//     OutputSourceData_t::DeliverySourceData::PubVisualizationMsgType_t doc_msg;
//     doc_msg.frame = frame_data->m_goal->frame;
//     output_source_data.set_document(doc_msg);

//     // create delivery request
//     auto delivery_request = _create_delivery_request(output_source_data);

//     // this is used for logging
//     auto msg_uuid = output_source_data.get_uuid();

//     // get qos, controls how to retry and drop frames
//     auto &qos = runtime_config->frame_enqueue_policy;
//     bool success = m_primary_output_port->push_request(delivery_request, qos);

//     if (success) {
//         RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
//                      "[msg_uuid={}] success to push request",
//                      boost::uuids::to_string(msg_uuid));
//     } else {
//         RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
//                      "[msg_uuid={}] failed to push request",
//                      boost::uuids::to_string(msg_uuid));
//     }

//     // FIXME: debug only
//     // wait for all requests to be processed, not necessary
//     m_primary_output_port->wait_for_all_requests();
// }

} // namespace redoxi_works
