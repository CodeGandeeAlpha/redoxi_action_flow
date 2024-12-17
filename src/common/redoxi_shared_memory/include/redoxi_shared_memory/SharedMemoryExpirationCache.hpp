#pragma once

#include <memory>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryTypes.hpp>

namespace redoxi_works::shared_memory
{

/**
 * @brief SharedMemoryExpirationCache is a cache that stores the expiration time of the shared memory blocks.
 * It is used to control the expiration of the shared memory blocks, and remove them from shm service when they are expired.
 */
class SharedMemoryExpirationCache
{
  public:
    SharedMemoryExpirationCache();
    ~SharedMemoryExpirationCache();


  private:
    struct Impl;
    std::weak_ptr<SharedMemoryClient> m_client;
};

} // namespace redoxi_works::shared_memory