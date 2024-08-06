#pragma once
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp>

#include <psg_actions/action/process_psg_document.hpp>
#include <psg_common/psg_common.hpp>
#include <psg_private_msgs/msg/psg_document.hpp>

#include <person_generator/person_generator_types.hpp>

namespace FlowRos2Pipeline
{
class PersonGeneratorImpl;

class PersonGenerator : public rclcpp::Node, public IOpenCloseProtocol
{
  public:
    class DownstreamFunctions
    {
      public:
        using DocumentAction = psg_actions::action::ProcessPsgDocument;
    };

    class DS_PSGDocument
    {
      public:
        virtual ~DS_PSGDocument()
        {
        }
        // client to call query service
        rclcpp_action::Client<DownstreamFunctions::DocumentAction>::SharedPtr handler;
        rclcpp_action::Client<DownstreamFunctions::DocumentAction>::SendGoalOptions options;
    };

    using InitConfig = PersonGeneratorInitConfig;
    using RuntimeConfig = PersonGeneratorRuntimeConfig;
    using MSG_Frame = psg_public_msgs::msg::Frame;
    using MSG_PSG_Doc = psg_private_msgs::msg::PsgDocument;

  public:
    explicit PersonGenerator();

    // initialize with configurations, must be called once before open()
    virtual int init(const std::shared_ptr<InitConfig> &config, const std::shared_ptr<RuntimeConfig> &runtime_config);

    // you can set configuration before open() or after close()
    virtual int update_init_config(const std::shared_ptr<InitConfig> &config);
    virtual const std::shared_ptr<InitConfig> &get_init_config() const;

    // modify runtime settings, must be called before start(), after stop() or close()
    virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> &config);
    virtual const std::shared_ptr<RuntimeConfig> &get_runtime_config() const;

    // can modify init config, runtime config

    // open video source, get ready to read
    virtual int open() override;

    // can modify runtime config

    // call this after ready() and before you spin this node
    // after calling this, you cannot modify runtime config
    virtual int start() override;

    // cannot modify any config, can call set_xxx() to modify relevant states

    // call this before you modify runtime config
    virtual int stop() override;

    // can modify runtime config

    // call this before you want to modify init config
    virtual int close() override;

    // can modify init config, runtime config

    // get the status code of this node
    virtual int get_status_code() const;

  protected:
    // downstream action handlers

    // psg document downstreams
    std::map<std::string, std::shared_ptr<DS_PSGDocument>> m_ds_psgdocument;
    // virtual void process_document_send_goals();
    virtual void process_document_goal_response_callback(
        const rclcpp_action::ClientGoalHandle<DownstreamFunctions::DocumentAction>::SharedPtr &goal_handle);
    virtual void process_document_feedback_callback(rclcpp_action::ClientGoalHandle<DownstreamFunctions::DocumentAction>::SharedPtr,
                                                    const std::shared_ptr<const DownstreamFunctions::DocumentAction::Feedback> feedback);
    virtual void process_document_result_callback(
        const rclcpp_action::ClientGoalHandle<DownstreamFunctions::DocumentAction>::WrappedResult &result);
    virtual void process_document_create_tasks(const MSG_Frame &frame);

    using GoalID = rclcpp_action::GoalUUID;
    class DSTask_PSGDocument
    {
      public:
        MSG_Frame frame; // frame associated with this task
        std::shared_ptr<DS_PSGDocument> downstream;
        GoalID goal_id; // id of the goal already sent to the downstream

        enum TaskStatus {
            TASK_NOT_SENT = 0,
            TASK_SENT = 1,
            TASK_DONE = 2,
            TASK_FAILED = 3,
        };
        TaskStatus status = TASK_NOT_SENT;
    };

  protected:
    virtual void _step();

    // find and connect to downstreams
    virtual void _connect_to_downstreams();

    // check if all downstreams are ready to accept new frame
    virtual bool _check_downstreams_ready();

    // send frame in shared memory to all downstreams
    // return whether the frame is actually sent
    virtual bool _send_PSG_document_to_downstreams(
        const MSG_PSG_Doc &psg_doc_msg,
        bool check_downstream_ready_before_send);

    virtual void _declare_all_parameters();

  protected:
    // configuration
    std::shared_ptr<InitConfig> m_init_config;
    std::shared_ptr<RuntimeConfig> m_runtime_config;

    // impl data
    std::shared_ptr<PersonGeneratorImpl> m_impl;

    // status code
    int m_status_code = NodeStatusCode::BEFORE_INIT;

    // publish info for visualization
    bool m_publish_image = false;

    // current frame number read by this reader
    // -1 means not read any frame, starting from 0 regardless of the absolute frame number in cv::VideoCapture
    int64_t m_frame_number = -1;
};
} // namespace FlowRos2Pipeline