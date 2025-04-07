#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_samples_nodes/generators/SimpleActionGenerator.hpp>
#include <redoxi_common_cpp/ros_utils/SyncActionSender.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseImpl.hpp>
#include <redoxi_samples_lib/random_image.hpp>

namespace redoxi_works
{

SimpleActionGenerator::SimpleActionGenerator(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase(name, options)
{
}

int SimpleActionGenerator::_read_frame(cv::Mat &frame, std::atomic<int64_t> &frame_number)
{
    //! Generate a random UUID and convert it to a string
    boost::uuids::random_generator gen;
    boost::uuids::uuid uuid = gen();
    std::string random_string = boost::uuids::to_string(uuid);

    //! Use the random string to generate the frame content
    random_image_with_text(frame, cv::Size(640, 480), random_string);
    frame_number++;

    return 0;
}

void SimpleActionGenerator::_step()
{
    _step_send_by_tbb_graph();
}

void SimpleActionGenerator::_step_send_by_tbb_graph()
{
    bool AlwaysUsePing = false;

    // read a frame
    cv::Mat frame;
    {
        auto ret = _read_frame(frame, m_frame_number);
        if (ret != 0) {
            RDX_LOG_ERROR(this, __func__, true, "{}", "Failed to read frame");
            return;
        }
    }

    FrameDeliveryTask_t frame_delivery_task;
    _create_frame_delivery_task(frame, frame_delivery_task);
    if (AlwaysUsePing) {
        // just use ping anyway
        redoxi_public_msgs::msg::Control control;
        control.code = control.PING;
        frame_delivery_task.control_signal = control;
    }

    //! Put the ping task into the frame delivery node
    auto msg_uuid = frame_delivery_task.uid;
    {
        auto ok = m_impl->frame_delivery_node->put_data(frame_delivery_task);
        if (!ok) {
            RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] put data FAILED", boost::uuids::to_string(msg_uuid));
        } else {
            RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] put data SUCCESS", boost::uuids::to_string(msg_uuid));
        }
    }

    for (const auto &downstream : m_downstreams) {
        auto frame_msg = this->_create_frame_message(frame_delivery_task, FrameDeliveryOptions_t::FramePayloadType::Uncompressed);
        _debug_publish_sent_to_downstream(frame_msg, downstream.second, 1, 1);
    }

    //! Wait for the frame delivery graph to finish
    // RDX_INFO_DEV(this, __func__, true, "waiting for frame delivery graph to finish");
    // m_impl->frame_delivery_graph->wait_for_all();
    // RDX_INFO_DEV(this, __func__, true, "frame delivery graph finished");
}

void SimpleActionGenerator::_step_send_by_sync_action_sender()
{
    std::chrono::milliseconds send_wait_time(100);
    for (const auto &downstream : m_downstreams) {
        //! Send a ping signal to the downstream using SyncActionSender
        SyncActionSender<Downstream_t::ActionType_t> sender(this);

        Downstream_t::Goal_t goal;
        goal.x_control.code = goal.x_control.PING;
        goal.x_uid = to_ros_uuid_msg(boost::uuids::random_generator()());

        auto result = sender.send(goal, *downstream.second->accept_frame, send_wait_time);

        if (result.response_code) {
            if (result.response_code.value() == ActionDownstreamResponse::ACCEPTED) {
                RDX_INFO_DEV(this, __func__, true, "Ping accepted by downstream: {}", downstream.first);
            } else if (result.response_code.value() == ActionDownstreamResponse::REJECTED) {
                RDX_INFO_DEV(this, __func__, true, "Ping rejected by downstream: {}", downstream.first);
            } else if (result.response_code.value() == ActionDownstreamResponse::TIMEOUT) {
                RDX_INFO_DEV(this, __func__, true, "Ping timed out for downstream: {}", downstream.first);
            }
        } else {
            RDX_INFO_DEV(this, __func__, true, "Unknown response from downstream: {}", downstream.first);
        }
    }
}

void SimpleActionGenerator::_step_send_and_block()
{
    // if not started yet, do nothing
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }
    // ping all downstreams
    for (const auto &downstream : m_downstreams) {
        RDX_INFO_DEV(this, __func__, true, "pinging downstream: {}", downstream.first);
        auto client = downstream.second->accept_frame;

        Downstream_t::Goal_t goal;
        Downstream_t::SendGoalOptions_t opt;
        opt.goal_response_callback = [this, downstream](const auto &goal_handle) {
            bool downstream_accepted = goal_handle != nullptr;
            RDX_INFO_DEV(this, __func__, true, "received goal response from downstream: {} (accepted: {})",
                         downstream.first, downstream_accepted);
        };
        opt.result_callback = [this, downstream](const auto &result) {
            RDX_INFO_DEV(this, __func__, true, "received result from downstream: {} (status: {}, goal_id: {})",
                         downstream.first, (int)result.code, to_boost_uuid_string(result.goal_id));
        };

        goal.x_control.code = goal.x_control.PING;
        goal.x_uid = to_ros_uuid_msg(boost::uuids::random_generator()());
        client->async_send_goal(goal, opt);
        RDX_INFO_DEV(this, __func__, true, "sent ping to downstream: {}", downstream.first);

        // block myself
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        RDX_INFO_DEV(this, __func__, true, "{}", "unblocked myself");
    }

    return;
}

} // namespace redoxi_works