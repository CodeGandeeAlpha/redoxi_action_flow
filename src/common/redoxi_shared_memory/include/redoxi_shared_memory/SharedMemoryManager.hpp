#pragma once

#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>

namespace redoxi_works::shared_memory
{

//! Keep a list of globally accessible shared memory clients
class SharedMemoryManager
{
  public:
    //! Get the singleton instance
    static SharedMemoryManager &get_instance();

    //! Get a shared memory client by config, create if not exists
    //! @return shared memory client pointer, nullptr if failed
    std::shared_ptr<SharedMemoryClient> get_client(const SharedMemoryConfig &config);

    //! remove a shared memory client by config
    //! @return 0 if success, -1 if failed (client not found)
    int remove_client(const SharedMemoryConfig &config);

    //! Get default client
    std::shared_ptr<SharedMemoryClient> get_default_client(rclcpp::Node *node = nullptr);

    //! Delete copy constructor and assignment operator
    SharedMemoryManager(const SharedMemoryManager &) = delete;
    SharedMemoryManager &operator=(const SharedMemoryManager &) = delete;

  private:
    //! Private constructor for singleton
    SharedMemoryManager() = default;

    //! Private destructor for singleton
    ~SharedMemoryManager() = default;

    //! Internal get_client implementation
    std::shared_ptr<SharedMemoryClient> _get_client(const SharedMemoryConfig &config);

    //! Map of region key to shared memory clients
    std::map<SharedMemoryConfig, std::shared_ptr<SharedMemoryClient>> m_clients_by_config;
};

} // namespace redoxi_works::shared_memory