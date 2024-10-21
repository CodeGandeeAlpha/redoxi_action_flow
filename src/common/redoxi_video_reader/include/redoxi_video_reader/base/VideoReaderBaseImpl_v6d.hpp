#pragma once

#include <redoxi_video_reader/base/VideoReaderBaseImpl.hpp>
#include <redoxi_common_cpp/redoxi_v6d.hpp>

namespace redoxi_works
{

class RedoxiVideoReaderImpl_v6d : public RedoxiVideoReaderImpl
{
  public:
    virtual ~RedoxiVideoReaderImpl_v6d() = default;
    RedoxiVideoReaderImpl_v6d(RedoxiVideoReaderBase *node)
        : RedoxiVideoReaderImpl(node)
    {
    }

    virtual int init_shared_memory_storage() override
    {
        return _init_v6d_client();
    }

    virtual int add_to_shared_memory(const cv::Mat &data, uint64_t &shared_memory_id) override
    {
        if (!v6d_client || !v6d_client->is_connected()) {
            RCLCPP_ERROR(ros_node->get_logger(), "[%s][_add_frame_to_shared_memory()] Vineyard client is not initialized or not connected", ros_node->get_name());
            return -1;
        }

        // add frame to vineyard shared memory
        auto ret = v6d_client->write_cvmat(data, shared_memory_id);
        if (ret != 0) {
            RCLCPP_ERROR(ros_node->get_logger(), "[%s][_add_frame_to_shared_memory()] Failed to add frame to shared memory", ros_node->get_name());
        }
        return ret;
    }

    virtual int remove_from_shared_memory(uint64_t shared_memory_id) override
    {
        if (!v6d_client || !v6d_client->is_connected()) {
            RCLCPP_ERROR(ros_node->get_logger(), "[%s][_remove_frame_from_shared_memory()] Vineyard client is not initialized or not connected", ros_node->get_name());
            return -1;
        }
        return v6d_client->delete_object(shared_memory_id);
    }

    // vineyard client, used to interact with vineyard shared memory
    std::shared_ptr<VineyardClient> v6d_client;

  private:
    // create vineyard client
    int _init_v6d_client()
    {
        // get parameters
        auto v6d_socket_name = ros_node->declare_parameter<std::string>(RosParams::Keys::v6d_socket_name, "");
        if (v6d_socket_name.empty()) {
            //! Vineyard socket name is not set
            RCLCPP_WARN(ros_node->get_logger(), "[%s][_init_v6d_client()] Vineyard socket name is not set, using default socket", ros_node->get_name());
        }

        // create vineyard client
        v6d_client = std::make_shared<VineyardClient>();

        // connect to vineyard server
        auto ret = v6d_client->connect(v6d_socket_name);
        if (!ret) {
            RCLCPP_ERROR(ros_node->get_logger(), "[%s][_init_v6d_client()] Failed to connect to vineyard server", ros_node->get_name());
            v6d_client = nullptr;
            return -1;
        }
        return 0;
    }
};

} // namespace redoxi_works
