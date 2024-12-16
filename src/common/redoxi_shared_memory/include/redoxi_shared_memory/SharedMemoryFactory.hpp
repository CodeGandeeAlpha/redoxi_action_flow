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
    static std::weak_ptr<SharedMemoryFactory> get_instance();

    //! Create a shared memory client based on the service type
    //! @return an uninitialized shared memory client, you need to connect it to the shared memory region before use, nullptr if failed
    std::shared_ptr<SharedMemoryClient>
        create_client_by_service_type(const std::string &service_type);

    //! Create a shared memory client based on the config
    //! @return a connected shared memory client, ready to use, nullptr if failed
    std::shared_ptr<SharedMemoryClient>
        create_client_by_config(const SharedMemoryConfig &config);

    //! Get the shared memory config from a ros node's parameters
    SharedMemoryConfig get_shm_config_from_node(const rclcpp::Node *node);

    //! Get the shared memory config from env variables
    SharedMemoryConfig get_shm_config_from_env();

    //! Delete copy constructor and assignment operator
    SharedMemoryFactory(const SharedMemoryFactory &) = delete;
    SharedMemoryFactory &operator=(const SharedMemoryFactory &) = delete;

    // allow to get the shared instance for lifetime management
    static std::shared_ptr<SharedMemoryFactory> _get_shared_instance() const;

  private:
    //! Private constructor for singleton
    SharedMemoryFactory() = default;

    //! Private destructor for singleton
    ~SharedMemoryFactory() = default;

    //! Get the shared memory service type from environment variables
    std::string get_shm_service_name_from_env();

    //! Get the shared memory region key from environment variables
    std::string get_shm_region_key_from_env();

    //! Singleton instance
    static std::shared_ptr<SharedMemoryFactory> s_instance;
};
} // namespace redoxi_works::shared_memory
