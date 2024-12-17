#pragma once

#include <memory>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryTypes.hpp>

namespace redoxi_works::shared_memory
{

/**
 * @brief ExpirationCache is a cache that stores the expiration time of the shared memory blocks.
 * It is used to control the expiration of the shared memory blocks, and remove them from shm service when they are expired.
 */
class ExpirationCache
{
  public:
    ExpirationCache(std::weak_ptr<SharedMemoryClient> client);

    // add a memory block to the cache
    // return 0 if success, -1 if failed
    int add_memory_block(const ObjectIdentifier &object_id,
                         const MemoryBlockExpirationConfig &expiration_config);

    // check and evict expired memory blocks
    // return the number of expired memory blocks
    int64_t evict_expired_memory_blocks();

  private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
    std::weak_ptr<SharedMemoryClient> m_client;
};

} // namespace redoxi_works::shared_memory