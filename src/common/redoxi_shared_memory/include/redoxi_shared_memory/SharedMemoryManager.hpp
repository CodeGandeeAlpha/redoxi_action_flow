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
    static std::shared_ptr<SharedMemoryClient> get_client(const SharedMemoryConfig &config);

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
    std::map<std::string, std::shared_ptr<SharedMemoryClient>> m_clients;
};

} // namespace redoxi_works::shared_memory