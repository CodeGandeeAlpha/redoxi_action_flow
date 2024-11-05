#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>

namespace redoxi_works
{
class SimpleActionGenerator : public RedoxiVideoReaderBase
{
  public:
    SimpleActionGenerator(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  protected:
    void _step() override;

    //! Implement _read_frame method
    int _read_frame(SourceData_t &source_data,
                    std::atomic<int64_t> &frame_number) override;


  private:
    //! test if the goal response can still be received while the sending thread is blocked
    //! conclusion: yes, it can
    void _step_send_and_block();

    //! send action by sync action sender, see if the goal response can still be received while the sending thread is blocked
    //! conclusion: yes, it can
    void _step_send_by_sync_action_sender();

    //! send by tbb graph
    void _step_send_by_tbb_graph();

  protected:
    //! Create delivery request
    rclcpp_action::Client<OutputPort_t::ActionType_t>::SharedPtr m_action_client;
};
} // namespace redoxi_works