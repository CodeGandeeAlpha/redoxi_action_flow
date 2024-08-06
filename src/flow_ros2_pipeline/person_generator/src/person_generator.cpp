#include <person_generator/_person_generator.hpp>
#include <person_generator/person_generator.hpp>

// namespace FlowRos2Pipeline {
//     PersonGenerator::PersonGenerator() : Node("master_node")
//     {
//         m_impl = std::make_shared<PersonGeneratorImpl>(this);

//         _declare_all_parameters();

//         RCLCPP_INFO(m_impl->logger, "[PersonGenerator] constraction success!");
//     }

//     void PersonGenerator::_declare_all_parameters() {
//         this->declare_parameter<std::string>("downstream_action", "");
//         this->declare_parameter<std::string>("status_query_service", "");
//         this->declare_parameter<std::string>("send_frame_action", "");
//         this->declare_parameter<double>("frame_internal_ms", -1);
//     }

//     int PersonGenerator::init(const std::shared_ptr<InitConfig>& config,
//                             const std::shared_ptr<RuntimeConfig>& runtime_config) {
//         if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
//             RCLCPP_ERROR(m_impl->logger, "[MasterNode] init FAILED! status code is not BEFORE_INIT or CLOSED");
//             return ReturnCode::ERROR;
//         }

//         m_init_config = config;
//         m_runtime_config = runtime_config;

//         m_memory_registry->connect_to_v6d("/var/run/vineyard.sock");


//         // create status_query server
//         std::string status_query_service = this->get_parameter("status_query_service").as_string();
//         // m_srv_status_query = this->create_service<MSG_StatusQuery>(
//         //     status_query_service, &MasterNode::status_query_callback);
//         m_srv_status_query = this->create_service<MSG_StatusQuery>(
//             status_query_service, std::bind(&MasterNode::status_query_callback, this, std::placeholders::_1, std::placeholders::_2));


//         // create send_frame server
//         std::string send_frame_action = this->get_parameter("send_frame_action").as_string();
//         m_act_accept_frame = rclcpp_action::create_server<ACT_AcceptFrame>(
//             this, send_frame_action,
//             std::bind(&MasterNode::accept_frame_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
//             std::bind(&MasterNode::accept_frame_cancel_callback, this, std::placeholders::_1),
//             std::bind(&MasterNode::accept_frame_accepted_callback, this, std::placeholders::_1));

//         // create downstream action client
//         m_ds_psgdocument["downstream_action"] = std::make_shared<DS_PSGDocument>();
//         m_ds_psgdocument["downstream_action"]->handler =
//                 rclcpp_action::create_client<DownstreamFunctions::DocumentAction>(this, m_init_config->downstream_action_name);
//         m_ds_psgdocument["downstream_action"]->options.goal_response_callback =
//                 std::bind(&MasterNode::process_document_goal_response_callback, this, std::placeholders::_1);
//         m_ds_psgdocument["downstream_action"]->options.feedback_callback =
//                 std::bind(&MasterNode::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
//         m_ds_psgdocument["downstream_action"]->options.result_callback =
//                 std::bind(&MasterNode::process_document_result_callback, this, std::placeholders::_1);

//         m_status_code = NodeStatusCode::INITIALIZED;
//         return ReturnCode::SUCCESS;
//     }

// }