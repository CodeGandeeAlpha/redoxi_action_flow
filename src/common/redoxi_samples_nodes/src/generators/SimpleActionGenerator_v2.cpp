#include <redoxi_samples_nodes/generators/SimpleActionGenerator_v2.hpp>
#include <redoxi_common_cpp/ros_utils/SyncActionSender.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_samples_lib/random_image.hpp>

namespace redoxi_works
{


SimpleActionGenerator_v2::SimpleActionGenerator_v2(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase_v2(name, options)
{
    // this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RDX_LOG_DEBUG(this, __func__, true, "{}", "create SimpleActionGenerator_v2");
}

int SimpleActionGenerator_v2::_read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number)
{
    //! Generate a random UUID and convert it to a string
    boost::uuids::random_generator gen;
    boost::uuids::uuid uuid = gen();
    std::string random_string = boost::uuids::to_string(uuid);

    //! Use the random string to generate the frame content
    cv::Mat frame;
    random_image_with_text(frame, cv::Size(640, 480), random_string);
    source_data.set_image(frame);
    source_data.set_frame_number(frame_number);
    frame_number++;

    return 0;
}

void SimpleActionGenerator_v2::_step()
{
    _step_send_by_tbb_graph();
}

void SimpleActionGenerator_v2::_step_send_by_tbb_graph()
{
    bool AlwaysUsePing = false;

    //! Read a frame
    SourceData_t source_data;
    {
        auto ret = _read_frame(source_data, m_frame_number);
        if (ret != 0) {
            RDX_LOG_ERROR(this, __func__, true, "{}", "Failed to read frame");
            return;
        }
    }

    //! Create delivery request
    auto delivery_request = _create_delivery_request(source_data);
    if (AlwaysUsePing) {
        delivery_request.as_ping();
    }

    //! Put the delivery request into the frame delivery node
    auto msg_uuid = delivery_request.get_source_data().get_uuid();
    {
        auto ok = m_primary_output_port->try_push_request(delivery_request);
        if (!ok) {
            RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] put data FAILED", boost::uuids::to_string(msg_uuid));
        } else {
            RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] put data SUCCESS", boost::uuids::to_string(msg_uuid));
        }
    }

    //! send it
    {
        RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] enqueue", boost::uuids::to_string(msg_uuid));
        auto ok = m_primary_output_port->try_push_request(delivery_request);
        if (!ok) {
            RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] enqueue FAILED", boost::uuids::to_string(msg_uuid));
        } else {
            RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] enqueue SUCCESS", boost::uuids::to_string(msg_uuid));
        }
    }

    //! Publish debug info if enabled
    // if (m_publish_to_debug_topic) {
    //     auto frame_msg = m_primary_output_port->create_frame_message(delivery_request);
    //     m_primary_output_port->debug_publish_sent_to_downstream(frame_msg, 1, 1);
    // }
}

void SimpleActionGenerator_v2::_step_send_and_block()
{
    //! Not implemented
}

void SimpleActionGenerator_v2::_step_send_by_sync_action_sender()
{
    //! Not implemented
}

} // namespace redoxi_works