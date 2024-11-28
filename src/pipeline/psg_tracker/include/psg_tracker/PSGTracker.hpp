#pragma once
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <redoxi_common_nodes/base_nodes/OpenCloseNode.hpp>
#include <psg_tracker/AsyncGetTrackTargetsInputPort.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <psg_tracker/PSGTrackerTypes.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include "psg_tracker/visibility_control.h"

namespace redoxi_works
{
struct PSGTrackerImpl;

class PSGTracker : public common_nodes::OpenCloseNode
{
    friend struct PSGTrackerImpl;
    friend struct psg_tracker::InitConfig;
    friend struct psg_tracker::RuntimeConfig;

  public:
    PSGTracker(const std::string &node_name, const rclcpp::NodeOptions &options);

  public: // useful types
    using InputPort_t = AsyncGetTrackTargetsInputPort;
    using SourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;
    using InitConfig_t = psg_tracker::InitConfig;
    using RuntimeConfig_t = psg_tracker::RuntimeConfig;

    using BaseInitConfig_t = common_nodes::OpenCloseNode::InitConfig_t;
    using BaseRuntimeConfig_t = common_nodes::OpenCloseNode::RuntimeConfig_t;

    //! Enable or disable debug topics
    void set_debug_topics_enabled(bool enable)
    {
        m_enable_debug_topics = enable;
    }

    //! Check if debug topics are enabled
    bool get_debug_topics_enabled() const
    {
        return m_enable_debug_topics;
    }

  protected:
    int _open() override;
    int _close() override;
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

    //! create implementation details of this node
    //! @note this must be called before any other operations, so it cannot access any member variables
    virtual std::shared_ptr<PSGTrackerImpl> _create_impl();

  protected:
    virtual int _parse_frame(cv::Mat *output, const SourceData_t &source_data);
    virtual std::vector<redoxi_public_msgs::msg::TrackTarget> _track(const std::shared_ptr<SourceData_t> &source_data,
                                                                     const ControlSignalCode &control_signal_code);

  protected:
    std::shared_ptr<InputPort_t> m_input_port;
    std::atomic<bool> m_enable_debug_topics{false};

    //! implementation details of this node
    std::shared_ptr<PSGTrackerImpl> m_impl;

    // publishers
    StampedImagePub m_pub_person_accepted;
    StampedImagePub m_pub_person_rejected;
    std::shared_ptr<shared_memory::SharedMemoryClient> m_shm_client;
};

} // namespace redoxi_works
