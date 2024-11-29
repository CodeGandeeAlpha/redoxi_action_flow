#include <redoxi_samples_nodes/generators/SimpleActionGenerator.hpp>
#include <redoxi_common_cpp/ros_utils/SyncActionSender.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_samples_lib/random_image.hpp>
#include <boost/timer/timer.hpp>

namespace redoxi_works
{


SimpleActionGenerator::SimpleActionGenerator(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase(name, options)
{
    // this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RDX_LOG_DEBUG(this, __func__, true, "{}", "create SimpleActionGenerator");
}

SimpleActionGenerator::ReadFrameResult
    SimpleActionGenerator::_read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number)
{
    // //! Generate a random UUID and convert it to a string
    // boost::uuids::random_generator gen;
    // boost::uuids::uuid uuid = gen();
    // std::string random_string = boost::uuids::to_string(uuid);

    // //! Use the random string to generate the frame content
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto frame_size = runtime_config->output_image_size;
    if (frame_size.empty()) {
        RDX_RAISE_ERROR("[{}][_read_frame()] output_image_size is not set", this->get_name());
    }

    //! Generate a random frame with the UUID text
    cv::Mat random_frame;
    auto uuid = source_data.get_uuid();
    auto frame_text = fmt::format("{}\nFrame Number: {}", boost::uuids::to_string(uuid), frame_number.load());
    random_image_with_text(random_frame, frame_size, frame_text);

    // must do it this way to ensure thread safety
    int64_t current_frame_number = _increment_frame_number_by(frame_number, 1);

    source_data.set_image(random_frame);
    SourceData_t::FrameMetadata_t metadata;
    metadata.frame_num = current_frame_number;
    metadata.width = random_frame.cols;
    metadata.height = random_frame.rows;
    source_data.set_frame_metadata(metadata);

    return ReadFrameResult::OK;
}

void SimpleActionGenerator::_step()
{
    // RedoxiVideoReaderBase::_step();
    _step_send_by_tbb_graph();
}

void SimpleActionGenerator::_step_send_by_tbb_graph()
{
    bool AlwaysUsePing = false;

    //! Read a frame
    SourceData_t source_data;
    {
        auto ret = _read_frame(source_data);
        if (ret != ReadFrameResult::OK) {
            RDX_LOG_ERROR(this, __func__, true, "{}", "Failed to read frame");
            return;
        }
    }

    //! Create delivery request
    auto delivery_request = _create_delivery_request(source_data);
    if (AlwaysUsePing) {
        delivery_request.set_control_signal_code(ControlSignalCode::Ping);
    }

    //! Put the delivery request into the frame delivery node
    auto msg_uuid = delivery_request.get_source_data().get_uuid();
    auto frame_num = delivery_request.get_source_data().get_frame_metadata().frame_num;
    // {
    //     auto ok = m_primary_output_port->try_push_request(delivery_request);
    //     if (!ok) {
    //         RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] put data FAILED", boost::uuids::to_string(msg_uuid));
    //     } else {
    //         RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] put data SUCCESS", boost::uuids::to_string(msg_uuid));
    //     }
    // }

    const auto &downstreams = m_primary_output_port->get_downstreams();
    for (const auto &downstream : downstreams) {
        RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] sending to downstream, frame_number={}",
                     boost::uuids::to_string(msg_uuid), frame_num);
        auto client = downstream.get_action_client();
        if (client) {
            auto start_time = std::chrono::high_resolution_clock::now();
            OutputPort_t::TargetData_t target_data;
            delivery_request.to_target_data(target_data);
            OutputPort_t::Downstream_t::ActionClient_t::SendGoalOptions opt;
            opt.result_callback = [this, msg_uuid](const auto &) {
                RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] goal result",
                             boost::uuids::to_string(msg_uuid));
            };
            auto send_result = client->async_send_goal(target_data.get_goal(), opt);
            {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] async_send_goal time: {} ms", boost::uuids::to_string(msg_uuid),
                             elapsed_time.count());
            }

            // wait for the result
            auto goal_handle = send_result.get();
            {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] get response time: {} ms", boost::uuids::to_string(msg_uuid),
                             elapsed_time.count());
            }

            if (goal_handle) {
                RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] goal accepted", boost::uuids::to_string(msg_uuid));
            } else {
                RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] goal send FAILED", boost::uuids::to_string(msg_uuid));
            }
        }
    }

    // auto publish_to_debug_topic = get_publish_to_debug_topic();

    //! Publish debug info if enabled
    // if (m_publish_to_debug_topic) {
    //     auto frame_msg = m_primary_output_port->create_frame_message(delivery_request);
    //     m_primary_output_port->debug_publish_sent_to_downstream(frame_msg, 1, 1);
    // }
}

void SimpleActionGenerator::_step_send_and_block()
{
    //! Not implemented
}

void SimpleActionGenerator::_step_send_by_sync_action_sender()
{
    //! Not implemented
}

} // namespace redoxi_works