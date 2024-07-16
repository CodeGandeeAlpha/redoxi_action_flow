#pragma once

#include <psg_common/psg_common.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <psg_actions/action/send_detections.hpp>
#include <psg_services/srv/status_query.hpp>

namespace FlowRos2Pipeline {
    class DetectorImpl;

    class Detector : public rclcpp::Node, public IOpenCloseProtocol {
    public:
        using DownstreamReadyQueryService = psg_services::srv::StatusQuery;
        using DownstreamSendDetectionsAction = psg_actions::action::SendDetections;  // TODO: need cache node or not?
        using DownstreamSendDetectionsActionGoalHandle = rclcpp_action::ClientGoalHandle<DownstreamSendDetectionsAction>;

        class Downstream{
        public:
            virtual ~Downstream(){}
            // client to call query service
            rclcpp::Client<DownstreamReadyQueryService>::SharedPtr get_status;
            rclcpp_action::Client<DownstreamSendDetectionsAction>::SharedPtr send_detections;
            // decltype(send_frame)::element_type::SendGoalOptions send_frame_options;
            rclcpp_action::Client<DownstreamSendDetectionsAction>::SendGoalOptions send_detections_options;
        };

        class InitConfig {
        public:
            virtual ~InitConfig(){}

            std::string status_query_service_name;
            std::string downstream_send_detections_action_name;
            std::string downstream_status_query_service_name;

            void from_parameters(Detector* node);
        };

        class RuntimeConfig {
        public:
            virtual ~RuntimeConfig(){}
            void from_parameters(Detector* node) {};
        };

    public:
        Detector();
        virtual ~Detector() {}

        // initialize with configurations, must be called once before open()
        virtual int init(const std::shared_ptr<InitConfig>& config, const std::shared_ptr<RuntimeConfig>& runtime_config);

        // you can set configuration before starting this node
        virtual int update_init_config(const std::shared_ptr<InitConfig> & config);
        const std::shared_ptr<InitConfig>& get_init_config() const;

        // modify runtime settings, must be called after open() or stop()
        virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> & config);
        const std::shared_ptr<RuntimeConfig>& get_runtime_config() const;

        // make the node ready to start, after calling this, you cannot modify init config
        virtual int open() override;

        // call this after ready() and before you spin this node
        // after calling this, you cannot modify runtime config
        virtual int start() override;

        // call this before you modify runtime config
        virtual int stop() override;

        // call this before you want to modify init config
        virtual int close() override;

        // get the status code of this node
        virtual int get_status_code() const;

    protected:
        void _declare_all_parameters();

        // member of downstreams
        std::map<std::string, std::shared_ptr<Downstream>> m_downstreams;

        // configuration
        std::shared_ptr<InitConfig> m_init_config;
        std::shared_ptr<RuntimeConfig> m_runtime_config;

        // impl data
        std::shared_ptr<DetectorImpl> m_impl;

        // status code
        int m_status_code = NodeStatusCode::BEFORE_INIT;
    };

}