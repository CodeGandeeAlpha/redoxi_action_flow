#pragma once

#include <memory>
#include <chrono>
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
    using TimeUnit_t = detail::TimeUnit_t;
    using TimePoint_t = detail::TimePoint_t;
    inline constexpr static TimeUnit_t DefaultCheckInterval = std::chrono::milliseconds(500);

    ExpirationCache(SharedMemoryClient *client);
    ~ExpirationCache();

    // add a memory block to the cache
    // return 0 if success, -1 if failed
    int add_memory_block(const ObjectIdentifier &object_id,
                         const ShmPutOptions &put_options);

    // check and evict expired memory blocks
    // return the number of expired memory blocks
    // if force_evict_all is true, evict all expired memory blocks
    int64_t evict_expired_memory_blocks(bool force_evict_all = false);

    // start the expiration cache thread
    // return 0 if success, -1 if failed
    int start_auto_evict(TimeUnit_t check_interval = DefaultCheckInterval);

    // stop the expiration cache thread
    // return 0 if success, -1 if failed
    int stop_auto_evict();

    // reset the expiration cache, evict all expired memory blocks
    void reset();

    // check if the auto eviction is enabled
    bool is_auto_evict_enabled() const
    {
        return m_auto_evict_running;
    }

  private:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
    SharedMemoryClient *m_client = nullptr;

    // auto eviction thread
    std::shared_ptr<std::thread> m_auto_evict_thread;
    std::atomic<bool> m_auto_evict_running{false};
};

} // namespace redoxi_works::shared_memory