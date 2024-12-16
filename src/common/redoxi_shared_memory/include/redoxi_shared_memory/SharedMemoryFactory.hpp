#pragma once

#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <json_struct/json_struct.h>
#include <rclcpp/node.hpp>

namespace redoxi_works::shared_memory
{

//! Factory class for creating shared memory clients based on different configurations
class SharedMemoryFactory
{
  public:
    //! Get the singleton instance
    static SharedMemoryFactory &get_instance();

    //! Get a shared memory client by config, create if not exists
    //! @return shared memory client pointer, nullptr if failed
    std::shared_ptr<SharedMemoryClient> get_client(const SharedMemoryConfig &config);

    //! Get default client from ros node's parameters or env variables
    std::shared_ptr<SharedMemoryClient> get_default_client(rclcpp::Node *node = nullptr);

    //! remove a shared memory client from the cache
    //! @return 0 if success, -1 if failed (client not found)
    int destroy_client(std::shared_ptr<SharedMemoryClient> client);

    //! Get the shared memory config from a ros node's parameters
    SharedMemoryConfig get_shm_config_from_node(const rclcpp::Node *node);

    //! Get the shared memory config from env variables
    SharedMemoryConfig get_shm_config_from_env();

    //! Get the default shared memory config from a ros node's parameters or env variables
    SharedMemoryConfig get_default_shm_config(const rclcpp::Node *node);

    //! Delete copy constructor and assignment operator
    // SharedMemoryFactory(const SharedMemoryFactory &) = delete;
    // SharedMemoryFactory &operator=(const SharedMemoryFactory &) = delete;
    // SharedMemoryFactory() = default;
    ~SharedMemoryFactory();

  private:
    struct Impl;

    //! Create a shared memory client based on the service type
    //! @return an uninitialized shared memory client, you need to connect it to the shared memory region before use, nullptr if failed
    std::shared_ptr<SharedMemoryClient>
        _create_client_by_service_type(const std::string &service_type);

    //! Create a shared memory client based on the config
    //! @return a connected shared memory client, ready to use, nullptr if failed
    std::shared_ptr<SharedMemoryClient>
        _create_client_by_config(const SharedMemoryConfig &config);

    //! Get the shared memory service type from environment variables
    std::string _get_shm_service_name_from_env();

    //! Get the shared memory region key from environment variables
    std::string _get_shm_region_key_from_env();

  private:
    // things to keep alive
    std::shared_ptr<Impl> m_impl;

    //! Map of region key to shared memory clients
    std::map<SharedMemoryConfig, std::shared_ptr<SharedMemoryClient>> m_clients_by_config;
};
} // namespace redoxi_works::shared_memory
