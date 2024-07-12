#pragma once

#include "psg_common/psg_common.hpp"
// #include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <psg_actions/action/send_frame.hpp>
#include <psg_services/srv/status_query.hpp>


namespace FlowRos2Pipeline{


    class OpencvVideoReaderImpl;

    /* Video reader node that reads video frames and sends them to downstreams,
    using cv::VideoCapture to read video frames, can accept any source that can be read by cv::VideoCapture.
    Supports reading partial video frames by specifying start and end frame numbers.
    */
    class OpencvVideoReader : public rclcpp::Node, public IOpenCloseProtocol {
        public:
            using DownstreamReadyQueryService = psg_services::srv::StatusQuery;
            using DownstreamSendFrameAction = psg_actions::action::SendFrame;
            using DownstreamSendFrameActionGoalHandle = rclcpp_action::ClientGoalHandle<DownstreamSendFrameAction>;

            class Downstream{
            public:
                virtual ~Downstream(){}
                // client to call query service
                rclcpp::Client<DownstreamReadyQueryService>::SharedPtr get_status;
                rclcpp_action::Client<DownstreamSendFrameAction>::SharedPtr send_frame;
                // decltype(send_frame)::element_type::SendGoalOptions send_frame_options;
                rclcpp_action::Client<DownstreamSendFrameAction>::SendGoalOptions send_frame_options;
            };

            class InitConfig{
            public:
                virtual ~InitConfig(){}
                // can be a file path or a camera index
                // only one source can be specified
                std::string source_file;
                int source_camera_index = -1; //-1 means not using camera

                //read frames as frames[start_frame_number:end_frame_number], like python
                int start_frame_number = 0; // 0 means start from the beginning
                int end_frame_number = -1;  // -1 means read all frames

                void from_parameters(OpencvVideoReader* node);
            };

            class RuntimeConfig{
            public:
                virtual ~RuntimeConfig(){}
                double frame_internal_ms = -1;
                int image_width = -1;
                int image_height = -1;
                void from_parameter(OpencvVideoReader* node);
            };

        public:
            // explicit VideoReader(const rclcpp::NodeOptions & options);
            explicit OpencvVideoReader();

            // initialize with configurations, must be called once before open()
            void init(const InitConfig& config, const RuntimeConfig& runtime_config);

            // you can set configuration before starting this node
            void update_init_config(const InitConfig& config);
            const InitConfig& get_init_config() const;

            // modify runtime settings, must be called after open() or stop()
            void update_runtime_config(const RuntimeConfig& config);
            const RuntimeConfig& get_runtime_config() const;

            // make the node ready to start, after calling this, you cannot modify init config
            virtual void open() override;

            // call this after ready() and before you spin this node
            // after calling this, you cannot modify runtime config
            virtual void start() override;

            // call this before you modify runtime config
            virtual void stop() override;

            // call this before you want to modify init config
            virtual void close() override;

            void img_read();

            void send_frame_goal_response_callback(const DownstreamSendFrameActionGoalHandle::SharedPtr & goal_handle);
            void send_frame_feedback_callback(DownstreamSendFrameActionGoalHandle::SharedPtr,
                                        const std::shared_ptr<const DownstreamSendFrameAction::Feedback> feedback);
            void send_frame_result_callback(const DownstreamSendFrameActionGoalHandle::WrappedResult & result);

            // void add_frame_goal_response_callback(const GoalHandleAddFrame::SharedPtr & goal_handle);
            // void add_frame_feedback_callback(GoalHandleAddFrame::SharedPtr,
            //                             const std::shared_ptr<const AddFrame::Feedback> feedback);
            // void add_frame_result_callback(const GoalHandleAddFrame::WrappedResult & result);

        protected:
            void _declare_all_parameters();

            // member of downstreams
            std::map<std::string, std::shared_ptr<Downstream>> m_downstreams;

            // configuration
            std::shared_ptr<InitConfig> m_init_config;
            std::shared_ptr<RuntimeConfig> m_runtime_config;

            // impl data
            std::shared_ptr<OpencvVideoReaderImpl> m_impl;

    };
}