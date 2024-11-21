#include <chrono>
#include <map>
#include <yolo8_body_pose_detector/Yolo8BodyPoseDetector.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>
#include <boost/thread/synchronized_value.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>

using DetectionMessage_t = redoxi_public_msgs::msg::Detection;
using PointMessage_t = geometry_msgs::msg::Point;
namespace redoxi_works::model_nodes
{

struct Yolo8BodyPoseDetector::Impl {
    tbb::task_group inference_task_group;
    // tbb::concurrent_queue<InferenceResource_t> inference_resources;
    tbb::concurrent_bounded_queue<InferenceResource_t> inference_resource_pool;
    boost::synchronized_value<std::map<GoalUUID_t, InferenceResource_t>> goal_to_inference_resource;

    // visualization publisher
    std::shared_ptr<StampedImagePub> pub_visualization;
};

Yolo8BodyPoseDetector::Yolo8BodyPoseDetector(const std::string &node_name,
                                             const rclcpp::NodeOptions &options)
    : redoxi_works::common_nodes::StartStopNode(node_name, options)
{
    m_impl = std::make_shared<Impl>();
}

Yolo8BodyPoseDetector::~Yolo8BodyPoseDetector() noexcept
{
    // do not call stop() here, because it will NOT call the subclass's stop()
    m_status = NodeStatusCode::STOPPED;

    if (m_step_thread && m_step_thread->joinable()) {
        m_step_thread->join();
    }

    // let all inference tasks finish
    m_impl->inference_task_group.wait();
}

#ifdef BIND_RESOURCE_TO_SOURCE_DATA
void Yolo8BodyPoseDetector::_register_input_port_callbacks(std::shared_ptr<ActionInputPort_t> input_port)
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

int Yolo8BodyPoseDetector::_start()
{
    // start the input port
    return m_input_port->start();
}

int Yolo8BodyPoseDetector::_extract_image(cv::Mat *output, const std::shared_ptr<ActionInputPort_t::SourceData_t> &source_data)
{
    if (!output) {
        // nothing to do
        return 0;
    }

    const auto &frame_msg = source_data->get_goal()->frame;

    // TODO: only support raw image, implement shared memory image extraction later
    const auto &raw_image = frame_msg.raw_image;
    *output = cv_bridge::toCvCopy(raw_image)->image;
    return 0;
}

int Yolo8BodyPoseDetector::_stop()
{
    // stop the input port from receiving new goals
    {
        auto ret = m_input_port->stop();
        if (ret != 0) {
            RDX_RAISE_ERROR("Failed to stop input port, error code: {}", ret);
        }
    }

    // wait for all inference tasks to finish
    m_impl->inference_task_group.wait();

    return 0;
}

int Yolo8BodyPoseDetector::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
{
    (void)runtime_config;
    return 0;
}

int Yolo8BodyPoseDetector::_update_init_config(std::shared_ptr<BaseInitConfig_t> init_config)
{
    auto config = std::dynamic_pointer_cast<InitConfig_t>(init_config);
    if (!config) {
        RDX_RAISE_ERROR("[f={}] Failed to convert init config to Yolo8BodyPoseDetector::InitConfig_t", __func__);
    }

    // create input port and init it
    m_input_port = std::make_shared<ActionInputPort_t>(this);
    {
        auto ret = m_input_port->init(config->input_port_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to init input port, error code: {}", __func__, ret);
        }
    }

    // create all inference resources
    {
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

void Yolo8BodyPoseDetector::_draw_visualization(cv::Mat &canvas,
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

void Yolo8BodyPoseDetector::_step()
{
    // do nothing if not started
    if (m_status != NodeStatusCode::STARTED) {
        return;
    }

    // FIXME: not very efficient, you will acquire resource frequently
    // but then just return it because no data is available

    // acquire a resource first
    InferenceResource_t inference_resource;
    bool acquired = m_impl->inference_resource_pool.try_pop(inference_resource);
    if (!acquired) {
        // no resource available, do not even try to get a message from input port
        return;
    }

    // get a message from input port
    auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto use_blocking_mode = config->enable_blocking_mode;
    std::shared_ptr<ActionInputPort_t::SourceData_t> source_data;
    if (use_blocking_mode) {
        source_data = m_input_port->pop_source_data();
    } else {
        source_data = m_input_port->try_pop_source_data();
    }
    if (!source_data) {
        // no data, return the resource
        m_impl->inference_resource_pool.push(inference_resource);
        return;
    }

    // bind the resource to the source data
    auto msg_uuid_str = boost::uuids::to_string(ActionDataTrait_t::get_uuid(*source_data->get_goal()));
    RDX_INFO_DEV(this, __func__, false,
                 "[msg_uid={}] Acquired a resource (index={}) and input data",
                 msg_uuid_str,
                 inference_resource.index_in_pool);

    // work on the image
    m_impl->inference_task_group.run([this, source_data, inference_resource, config]() {
        // reply to the goal
        auto msg_uuid_str = boost::uuids::to_string(ActionInputPort_t::ActionDataTrait_t::get_uuid(*source_data->get_goal()));
        auto goal_handle = source_data->get_goal_handle_future().get();
        if (!goal_handle) {
            // nothing to do, just return the resource
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Goal handle not found, return the resource", msg_uuid_str);
            m_impl->inference_resource_pool.push(inference_resource);
            return;
        }

        // extract image
        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Extracting image", msg_uuid_str);
        cv::Mat image;
        auto ret = _extract_image(&image, source_data);
        if (ret != 0) {
            RDX_RAISE_ERROR("Failed to extract image");
        }

        if (image.empty()) {
            // empty image, nothing to do
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Empty image, nothing to do", msg_uuid_str);

            // return the resource
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Returning the resource", msg_uuid_str);
            m_impl->inference_resource_pool.push(inference_resource);

            // reply to the goal
            RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Aborting the goal", msg_uuid_str);
            goal_handle->abort(std::make_shared<InputAction_t::Result>());
            return;
        }

        auto model = inference_resource.model;
        auto inout_data = inference_resource.inout_data;

        // inference
        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Doing inference", msg_uuid_str);
        model->set_input_images(inout_data, {image}, RequiredImageEncoding);
        model->do_inference(inout_data);

        // get result
        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Getting inference result", msg_uuid_str);
        auto output_detections = model->get_output_detections(inout_data,
                                                              config->model_output_config);

        // DO NOT return the resource here, although we do not need it anymore
        // if goal reply is slower than inference (although unlikely), we will have a lot of pending reply requests
        // m_impl->inference_resource_pool.push(inference_resource);

        // reply
        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Replying to the goal as success", msg_uuid_str);
        auto goal_reply = std::make_shared<InputAction_t::Result>();
        for (size_t i = 0; i < output_detections[0].objects.size(); i++) {
            auto &obj = output_detections[0].objects[i];
            DetectionMessage_t detection_msg;
            detection_msg.category = obj.class_id;
            detection_msg.category_name = obj.class_name;
            detection_msg.confidence = obj.score;
            detection_msg.bbox.x = obj.xywh[0];
            detection_msg.bbox.y = obj.xywh[1];
            detection_msg.bbox.width = obj.xywh[2];
            detection_msg.bbox.height = obj.xywh[3];
            detection_msg.frame_metadata = source_data->get_goal()->frame.metadata;

            // Fill keypoints
            for (size_t k = 0; k < obj.keypoints.size(); k++) {
                const auto &kp = obj.keypoints[k];
                PointMessage_t point_msg;
                point_msg.x = kp.xy[0];
                point_msg.y = kp.xy[1];
                detection_msg.keypoints.keypoints_2.push_back(point_msg);
                detection_msg.keypoints.confidence.push_back(kp.score);
                detection_msg.keypoints.semantic_type.push_back(k);
            }
            goal_reply->detections.push_back(detection_msg);
        }
        goal_handle->succeed(goal_reply);

        // visualize?
        if (m_impl->pub_visualization && config->enable_visualization) {
            cv::Mat vis_canvas = image.clone();
            _draw_visualization(vis_canvas, output_detections[0]);
            m_impl->pub_visualization->publish(vis_canvas);
        }

        // we can return the resource now
        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Returning the resource", msg_uuid_str);
        m_impl->inference_resource_pool.push(inference_resource);

        RDX_INFO_DEV(this, __func__, false, "[msg_uid={}] Inference and replay done", msg_uuid_str);
    });
}

int Yolo8BodyPoseDetector::_create_inference_resource(
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

int Yolo8BodyPoseDetector::_create_all_inference_resources(
    const std::vector<InitConfig_t::ModelConfig_t::Ptr> &model_configs)
{
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

} // namespace redoxi_works::model_nodes
