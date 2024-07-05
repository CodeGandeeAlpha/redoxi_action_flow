#pragma once

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>
#include <memory>
#include <vineyard/client/client.h>
#include "my_srvs/srv/RegisterObjId.hpp"

namespace FlowRos2Pipeline{

    class CacheNode : public rclcpp::Node {
    public:
        CacheNode();
        void register_obj_id(const std::shared_ptr<my_srvs::srv::RegisterObjId::Request> request,
                std::shared_ptr<my_srvs::srv::RegisterObjId::Response> response);
        void get_obj_id(const std::shared_ptr<my_srvs::srv::RegisterObjId::Request> request,
                std::shared_ptr<my_srvs::srv::RegisterObjId::Response> response);
        void delete_obj_id(const std::shared_ptr<my_srvs::srv::RegisterObjId::Request> request,
            std::shared_ptr<my_srvs::srv::RegisterObjId::Response> response);
    private:
        std::map<int, std::map<std::string, std::string>> m_map_obj_id;

        rclcpp::Service<my_srvs::srv::RegisterObjId>::SharedPtr m_server_register_obj_id;
        rclcpp::Service<my_srvs::srv::DeleteObjId>::SharedPtr m_server_delete_obj_id;
        rclcpp::Service<my_srvs::srv::GetObjId>::SharedPtr m_server_get_obj_id;

        std::shared_ptr<vineyard::Client> m_v6d_client;
        rclcpp::Logger logger_;
    };

}

#endif // CACHE_HPP_