#include "cache_node/cache.hpp"

namespace FlowROS2Pipeline {
    CacheNode::CacheNode() : rclcpp::Node("cache_node"), logger_(this->get_logger()) {
        // 声明参数
        this->declare_parameter<std::string>("register_obj_id_srv", "");
        this->declare_parameter<std::string>("get_obj_id_srv", "");
        this->declare_parameter<std::string>("delete_obj_id_srv", "");
        this->declare_parameter<std::string>("v6d_ipc_socket", "");

        // 获取参数
        std::string register_obj_id_srv = this->get_parameter("register_obj_id_srv").as_string();
        std::string get_obj_id_srv = this->get_parameter("get_obj_id_srv").as_string();
        std::string delete_obj_id_srv = this->get_parameter("delete_obj_id_srv").as_string();
        std::string v6d_ipc_socket = this->get_parameter("v6d_ipc_socket").as_string();
        RCLCPP_INFO(logger_, "[CacheNode] register_obj_id_srv: %s", register_obj_id_srv.c_str());
        RCLCPP_INFO(logger_, "[CacheNode] get_obj_id_srv: %s", get_obj_id_srv.c_str());
        RCLCPP_INFO(logger_, "[CacheNode] delete_obj_id_srv: %s", delete_obj_id_srv.c_str());
        RCLCPP_INFO(logger_, "[CacheNode] v6d_ipc_socket: %s", v6d_ipc_socket.c_str());

        // v6d init
        m_v6d_client = std::make_shared<vineyard::Client>();
        m_v6d_client->Connect(v6d_ipc_socket);

        // 创建service server
        m_server_register_obj_id = this->create_service<my_srvs::srv::RegisterObjId>(register_obj_id_srv, &CacheNode::register_obj_id);
        m_server_get_obj_id = this->create_service<my_srvs::srv::GetObjId>(get_obj_id_srv, &CacheNode::get_obj_id);
        m_server_delete_obj_id = this->create_service<my_srvs::srv::DeleteObjId>(delete_obj_id_srv, &CacheNode::delete_obj_id);

        RCLCPP_INFO(logger_, "[CacheNode] init success!");
    }

    void CacheNode::register_obj_id(const std::shared_ptr<my_srvs::srv::RegisterObjId::Request> request,
                std::shared_ptr<my_srvs::srv::RegisterObjId::Response> response) {
        // 检查外层map是否存在frame_num
        auto outer_iter = m_map_obj_id.find(request->frame_num);
        if (outer_iter != m_map_obj_id.end()) {
            // 外层map存在frame_num
            auto& inner_map = outer_iter->second;

            // 检查内层map是否存在request->meaning
            auto inner_iter = inner_map.find(request->meaning);
            if (inner_iter != inner_map.end()) {
                // 内层map存在request->meaning，更新值
                inner_iter->second = value;
            } else {
                // 内层map不存在request->meaning，添加新值
                inner_map[request->meaning] = value;
            }
        } else {
            // 外层map不存在request->frame_num，添加新的外层和内层键值对
            m_map_obj_id[request->frame_num][request->meaning] = value;
        }

        // return
        response->flag = "ok";
        response->return_code = 0;
    }

    void CacheNode::get_obj_id(const std::shared_ptr<my_srvs::srv::GetObjId::Request> request,
                std::shared_ptr<my_srvs::srv::GetObjId::Response> response) {
        response->obj_id = "";
        response->flag = "obj not exist";
        response->return_code = 1;

        auto outer_iter = m_map_obj_id.find(request->frame_num);
        if (outer_iter != m_map_obj_id.end()) {
            // 外层map存在frame_num
            auto& inner_map = outer_iter->second;

            // 检查内层map是否存在request->meaning
            auto inner_iter = inner_map.find(request->meaning);
            if (inner_iter != inner_map.end()) {
                // 内层map存在request->meaning
                response->obj_id = inner_iter->second;
                response->flag = "ok";
                response->return_code = 0;
            }
        }
    }

    void CacheNode::delete_obj_id(const std::shared_ptr<my_srvs::srv::GetObjId::Request> request,
                std::shared_ptr<my_srvs::srv::GetObjId::Response> response) {
        response->obj_id = "";
        response->flag = "obj not exist";
        response->return_code = 1;

    }
}