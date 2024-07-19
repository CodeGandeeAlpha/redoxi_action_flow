#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <psg_common/psg_common.hpp>
#include <psg_actions/action/send_frame.hpp>
#include <psg_services/srv/status_query.hpp>

#include <video_reader/video_reader_types.hpp>


namespace FlowRos2Pipeline{
    class OpencvVideoReaderImpl;

    /* Video reader node that reads video frames and sends them to downstreams,
    using cv::VideoCapture to read video frames, can accept any source that can be read by cv::VideoCapture.
    Supports reading partial video frames by specifying start and end frame numbers.
    */
    class OpencvVideoReader : public rclcpp::Node, public IOpenCloseProtocol {
        public:
            using DownstreamReadyQueryService = psg_services::srv::StatusQuery;
            using DownstreamAcceptFrameAction = psg_actions::action::SendFrame;
            using DownstreamSendFrameActionGoalHandle = rclcpp_action::ClientGoalHandle<DownstreamAcceptFrameAction>;

            class Downstream{
            public:
                virtual ~Downstream(){}
                // client to call query service
                rclcpp::Client<DownstreamReadyQueryService>::SharedPtr get_status;
                rclcpp_action::Client<DownstreamAcceptFrameAction>::SharedPtr accept_frame;
                rclcpp_action::Client<DownstreamAcceptFrameAction>::SendGoalOptions accept_frame_options;
            };

            using InitConfig = OpencvVideoReaderInitConfig;
            using RuntimeConfig = OpencvVideoReaderRuntimeConfig;
            using MSG_Frame = psg_public_msgs::msg::Frame;
            using MSG_IMG = sensor_msgs::msg::Image;

            inline static const std::string TOPIC_IMAGE = "image";
            inline static const int DEFAULT_IMAGE_TOPIC_QUEUE_LENGTH = 10;

        public:
            explicit OpencvVideoReader();

            virtual int set_image_topic_enable(bool enable);
            virtual std::string get_image_topic_name() const;

            // initialize with configurations, must be called once before open()
            virtual int init(const std::shared_ptr<InitConfig>& config, const std::shared_ptr<RuntimeConfig>& runtime_config);

            // you can set configuration before open() or after close()
            virtual int update_init_config(const std::shared_ptr<InitConfig> & config);
            virtual const std::shared_ptr<InitConfig>& get_init_config() const;

            // modify runtime settings, must be called before start(), after stop() or close()
            virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> & config);
            virtual const std::shared_ptr<RuntimeConfig>& get_runtime_config() const;

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
            virtual void _step();

            // create publisher for visualization
            virtual void _create_image_topic();

            // find and connect to downstreams
            virtual void _connect_to_downstreams();

            // check if all downstreams are ready to accept new frame
            virtual bool _check_downstreams_ready();

            // add a frame to shared memory, return object id
            virtual uint64_t _add_frame_to_shared_memory(const cv::Mat& frame);

            // send frame in shared memory to all downstreams
            // return whether the frame is actually sent
            virtual bool _send_frame_to_downstreams(
                const MSG_Frame& frame_msg,
                bool check_downstream_ready_before_send);

            virtual void _declare_all_parameters();

            // read next frame and return true if success
            virtual bool _read_frame(cv::Mat& frame);

            // publish frame msg for visualization
            virtual void _publish_frame(const cv::Mat& frame);

        protected:
            // member of downstreams
            std::map<std::string, std::shared_ptr<Downstream>> m_downstreams;

            // configuration
            std::shared_ptr<InitConfig> m_init_config;
            std::shared_ptr<RuntimeConfig> m_runtime_config;

            // impl data
            std::shared_ptr<OpencvVideoReaderImpl> m_impl;

            // status code
            int m_status_code = NodeStatusCode::BEFORE_INIT;

            // publish info for visualization
            bool m_publish_image = false;
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_topic_image;

            // current frame number read by this reader
            // -1 means not read any frame, starting from 0 regardless of the absolute frame number in cv::VideoCapture
            int64_t m_frame_number = -1;
    };
}