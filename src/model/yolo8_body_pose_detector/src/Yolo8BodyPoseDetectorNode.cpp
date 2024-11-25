#include <chrono>
#include <map>

#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>
#include <boost/thread/synchronized_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <yolo8_body_pose_detector/Yolo8BodyPoseDetectorNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_dnn_models/message_conversion.hpp>

using DetectionMessage_t = redoxi_public_msgs::msg::Detection;
using PointMessage_t = geometry_msgs::msg::Point;
namespace redoxi_works::model_nodes
{

struct Yolo8BodyPoseDetectorNode::Impl {
    tbb::task_group inference_task_group;
    // tbb::concurrent_queue<InferenceResource_t> inference_resources;
    tbb::concurrent_bounded_queue<InferenceResource_t> inference_resource_pool;

    // visualization publisher
    std::shared_ptr<StampedImagePub> pub_visualization;
};

Yolo8BodyPoseDetectorNode::Yolo8BodyPoseDetectorNode(const std::string &node_name,
                                                     const rclcpp::NodeOptions &options)
    : redoxi_works::common_nodes::StartStopNode(node_name, options)
{
    m_impl = std::make_shared<Impl>();
}

Yolo8BodyPoseDetectorNode::~Yolo8BodyPoseDetectorNode() noexcept
{
    // do not call stop() here, because it will NOT call the subclass's stop()
    m_status = NodeStatusCode::STOPPED;

    if (m_step_thread && m_step_thread->joinable()) {
        m_step_thread->join();
    }

    // let all inference tasks finish
    m_impl->inference_task_group.wait();
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
        RDX_RAISE_ERROR("[f={}] Empty image data", __func__);
    }
    *output = cv_bridge::toCvCopy(raw_image)->image;
    return 0;
}

//! Process image request
int Yolo8BodyPoseDetectorNode::_process_image_request()
{
    if (!m_image_request_input_port) {
        RDX_INFO_DEV(this, __func__, false, "{}", "No image request input port, skipping");
        return 0;
    }

    using InputDataTrait_t = ByImageRequest::InputPort_t::ActionDataTrait_t;
    using InputAction_t = ByImageRequest::InputAction_t;
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto use_blocking_mode = runtime_config->enable_blocking_mode;

    std::shared_ptr<ByImageRequest::InputSourceData_t> input_data;
    if (use_blocking_mode) {
        input_data = m_image_request_input_port->pop_source_data();
    } else {
        input_data = m_image_request_input_port->try_pop_source_data();
    }
    if (!input_data) {
        //! No data available
        return 0;
    }

    // used for logging
    auto msg_uuid = InputDataTrait_t::get_uuid(*input_data->get_goal());
    std::string msg_uuid_str = UUIDTrait::to_string(msg_uuid);

    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Got an image request from input port", msg_uuid_str);

    // got data, wait for goal handle
    auto goal_handle = input_data->get_goal_handle_future().get();
    if (!goal_handle) {
        RDX_RAISE_ERROR("[f={}] Goal handle not found, something unexpected happened", __func__);
    }

    //! Extract image from input data
    cv::Mat input_image;
    {
        auto ret = _extract_image(&input_image, input_data);
        if (ret != 0) {
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Failed to extract image from input data, aborting goal", msg_uuid_str);
            goal_handle->abort(std::make_shared<InputAction_t::Result>());
            return -1;
        }
    }

    //! Try to get an inference resource
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Getting an inference resource", msg_uuid_str);
    InferenceResource_t resource;
    {
        auto ret = m_impl->inference_resource_pool.pop(resource);
        if (ret != 0) {
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Failed to get an inference resource, aborting goal", msg_uuid_str);
            goal_handle->abort(std::make_shared<InputAction_t::Result>());
            return -1;
        }
    }

    // do inference
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Got an inference resource, do inference", msg_uuid_str);
    DetectionResult_t det_result;
    auto ret = _do_inference(&det_result, input_image, resource, msg_uuid);
    {
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to do inference, error code: {}", __func__, ret);
        }
    }

    // create output action and send it to output port
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Creating output action and sending it to output port", msg_uuid_str);
    if (m_image_request_output_port) {
        auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
        ByImageRequest::OutputSourceData_t o_source;
        o_source.image = input_image;
        inference::conversion::to_ros_msg(&o_source.detections, det_result);

        ByImageRequest::OutputRequest_t request;
        request.set_source_data(o_source);

        const auto &output_enqueue_policy = init_config->image_request_config->output_enqueue_policy;
        auto pushed_ok = m_image_request_output_port->push_request(request, output_enqueue_policy);
        if (pushed_ok) {
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Pushed target data to image request output port", msg_uuid_str);
        } else {
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Failed to push target data to image request output port", msg_uuid_str);
        }
    } else {
        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] No image request output port, skipping", msg_uuid_str);
    }

    //! Schedule inference task
    return 0;
}

int Yolo8BodyPoseDetectorNode::_do_inference(DetectionResult_t *output_result,
                                             const cv::Mat &input_image,
                                             const InferenceResource_t &resource,
                                             std::optional<UUIDType> msg_uuid)
{
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

    // wait for all inference tasks to finish
    m_impl->inference_task_group.wait();

    return 0;
}

int Yolo8BodyPoseDetectorNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
{
    (void)runtime_config;
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
    if (!m_detection_request_input_port) {
        RDX_INFO_DEV(this, __func__, false, "{}", "No detection request input port, skipping");
        return 0;
    }

    using ActionDataTrait_t = ByDetectionRequest::InputPort_t::ActionDataTrait_t;
    using InputAction_t = ByDetectionRequest::InputAction_t;

    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto use_blocking_mode = runtime_config->enable_blocking_mode;

    std::shared_ptr<ByDetectionRequest::InputSourceData_t> input_data;
    if (use_blocking_mode) {
        input_data = m_detection_request_input_port->pop_source_data();
    } else {
        input_data = m_detection_request_input_port->try_pop_source_data();
    }
    if (!input_data) {
        //! No data available
        return 0;
    }

    // used for logging
    auto msg_uuid = ActionDataTrait_t::get_uuid(*input_data->get_goal());
    std::string msg_uuid_str = UUIDTrait::to_string(msg_uuid);

    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Got a detection request from input port", msg_uuid_str);

    // got data, wait for goal handle
    auto goal_handle = input_data->get_goal_handle_future().get();
    if (!goal_handle) {
        RDX_RAISE_ERROR("[f={}] Goal handle not found, something unexpected happened", __func__);
    }

    //! Extract image from input data
    cv::Mat input_image;
    {
        auto ret = _extract_image(&input_image, input_data);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to extract image from input data, error code: {}", __func__, ret);
        }
    }

    if (input_image.empty()) {
        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Empty image, aborting goal", msg_uuid_str);
        goal_handle->abort(std::make_shared<InputAction_t::Result>());
        return -1;
    }

    //! Try to get an inference resource
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Getting an inference resource", msg_uuid_str);
    InferenceResource_t resource;
    {
        auto ret = m_impl->inference_resource_pool.pop(resource);
        if (ret != 0) {
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Failed to get an inference resource, aborting goal", msg_uuid_str);
            goal_handle->abort(std::make_shared<InputAction_t::Result>());
            return -1;
        }
    }

    // do inference
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Got an inference resource, do inference", msg_uuid_str);
    DetectionResult_t det_result;
    auto ret = _do_inference(&det_result, input_image, resource, msg_uuid);
    {
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to do inference, error code: {}", __func__, ret);
        }
    }

    // create result and reply
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Creating result and replying", msg_uuid_str);
    auto goal_reply = std::make_shared<InputAction_t::Result>();

    // Convert detections using message conversion utilities
    inference::conversion::to_ros_msg(&goal_reply->detections, det_result);

    // Add frame metadata to each detection
    for (auto &detection : goal_reply->detections) {
        detection.frame_metadata = input_data->get_goal()->frame.metadata;
    }

    // send it
    goal_handle->succeed(goal_reply);

    // visualize if enabled
    if (m_impl->pub_visualization && runtime_config->enable_visualization) {
        cv::Mat vis_canvas = input_image.clone();
        _draw_visualization(vis_canvas, det_result);
        m_impl->pub_visualization->publish(vis_canvas);
    }

    // return the resource
    RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Returning inference resource", msg_uuid_str);
    m_impl->inference_resource_pool.push(resource);

    return 0;
}

void Yolo8BodyPoseDetectorNode::_step()
{
    // do nothing if not started
    if (m_status != NodeStatusCode::STARTED) {
        return;
    }

    {
        RDX_INFO_DEV(this, __func__, false, "{}", "Processing detection request");
        auto ret = _process_detection_request();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to process detection request, error code: {}", __func__, ret);
        }
    }
    {
        RDX_INFO_DEV(this, __func__, false, "{}", "Processing image request");
        auto ret = _process_image_request();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to process image request, error code: {}", __func__, ret);
        }
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
    return 0;
}

#ifdef BIND_RESOURCE_TO_SOURCE_DATA
void Yolo8BodyPoseDetectorNode::_register_input_port_callbacks(std::shared_ptr<ActionInputPort_t> input_port)
{
    // resource booking callback
    input_port->set_on_goal_received_callback(
        [this](const auto &goal_uuid, const auto &) {
            // try to book a resource for the goal
            InferenceResource_t inference_resource;
            bool booked = m_impl->inference_resource_pool.try_pop(inference_resource);
            if (booked) {
                // book the resource, and accept the goal
                m_impl->goal_to_inference_resource->insert({goal_uuid, inference_resource});
                return 0;
            }
            // no resource available, reject the goal
            return -1;
        });

    // bind the resource to the source data on accept
    input_port->set_on_goal_enqueued_callback(
        [this](std::shared_ptr<SourceData_t> source_data) {
            // lock the goal to inference resource
            auto lock = m_impl->goal_to_inference_resource.synchronize();

            // get the booked resource
            auto inference_resource = lock->find(source_data->get_goal_uuid());

            // should not happen, the resource should be booked before enqueued
            if (inference_resource == lock->end()) {
                RDX_RAISE_ERROR("No booked resource found for goal uuid: {}", to_boost_uuid_string(source_data->get_goal_uuid()));
            }

            // bind the resource to the source data
            source_data->auxiliary_data = inference_resource;

            // release the book keeping entry
            lock->erase(source_data->get_goal_uuid());

            return 0;
        });

    // remove booked resource if goal is rejected or canceled
    input_port->set_on_goal_rejected_callback(
        [this](const auto &goal_uuid, auto) {
            auto lock = m_impl->goal_to_inference_resource.synchronize();
            auto it = lock->find(goal_uuid);
            if (it != lock->end()) {
                // return resource to pool
                m_impl->inference_resource_pool.push(it->second);
                // remove from book keeping
                lock->erase(it);
            }
            return 0;
        });

    input_port->set_on_goal_cancel_request_callback(
        [this](auto goal_handle) {
            auto lock = m_impl->goal_to_inference_resource.synchronize();
            auto it = lock->find(goal_handle->get_goal_id());
            if (it != lock->end()) {
                // return resource to pool
                m_impl->inference_resource_pool.push(it->second);
                // remove from book keeping
                lock->erase(it);
            }
            return 0;
        });
}
#endif

} // namespace redoxi_works::model_nodes
