#include <rclcpp/rclcpp.hpp>
#include <redoxi_shared_memory/ExpirationCache.hpp>
#include <boost/bimap.hpp>
#include <boost/bimap/multiset_of.hpp>
#include <boost/bimap/support/lambda.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <mutex>

namespace bimaps = boost::bimaps;

namespace redoxi_works::shared_memory
{

struct DataBlockInfo {
    using TimePoint_t = detail::TimePoint_t;
    using TimeUnit_t = detail::TimeUnit_t;

    DataBlockInfo()
        : time_created(std::chrono::system_clock::now())
    {
    }

    DataBlockInfo(const ObjectIdentifier &object_id,
                  const ShmPutOptions &put_options)
        : object_id(object_id),
          put_options(put_options),
          time_created(std::chrono::system_clock::now())
    {
        if (put_options.alive_duration.has_value()) {
            time_to_evict = time_created + put_options.alive_duration.value();
        }
    }

    ObjectIdentifier object_id;
    ShmPutOptions put_options;
    TimePoint_t time_created;
    TimePoint_t time_to_evict = TimePoint_t::max(); // default to no expiration

    //! Comparison operators for use in std::map
    bool operator<(const DataBlockInfo &other) const
    {
        return time_to_evict < other.time_to_evict;
    }

    bool operator==(const DataBlockInfo &other) const
    {
        return time_to_evict == other.time_to_evict;
    }

    bool operator!=(const DataBlockInfo &other) const
    {
        return !(*this == other);
    }

    //! Compare with a time point to check if this block is expired
    bool operator<=(const TimePoint_t &time_point) const
    {
        return time_to_evict <= time_point;
    }

    bool operator>=(const TimePoint_t &time_point) const
    {
        return time_to_evict >= time_point;
    }

    bool operator>(const TimePoint_t &time_point) const
    {
        return time_to_evict > time_point;
    }

    bool operator<(const TimePoint_t &time_point) const
    {
        return time_to_evict < time_point;
    }
};

struct ExpirationCache::Impl {
    bimaps::bimap<bimaps::set_of<ObjectIdentifier>,
                  bimaps::multiset_of<DataBlockInfo>>
        m_cache;

    // mutex for cache operations
    std::mutex m_cache_mutex;
};

ExpirationCache::ExpirationCache(SharedMemoryClient *client)
    : m_impl(std::make_shared<Impl>()), m_client(client)
{
    // register shutdown callback, on shutdown, stop the auto eviction thread and evict all expired memory blocks
    rclcpp::on_shutdown(std::function<void()>([this]() {
        reset();
    }));
}

void ExpirationCache::reset()
{
    stop_auto_evict();
    evict_expired_memory_blocks(true);
}

ExpirationCache::~ExpirationCache()
{
    reset();
}

int ExpirationCache::start_auto_evict(TimeUnit_t check_interval)
{
    if (m_auto_evict_thread && m_auto_evict_running) {
        RDX_WARN_DEV(nullptr, __func__, "{}", "Auto eviction thread already running");
        return -1;
    }

    m_auto_evict_running = true;
    m_auto_evict_thread = std::make_shared<std::thread>([this, check_interval]() {
        while (m_auto_evict_running) {
            evict_expired_memory_blocks();
            std::this_thread::sleep_for(check_interval);
        }
    });
    return 0;
}

int ExpirationCache::stop_auto_evict()
{
    m_auto_evict_running = false;
    if (m_auto_evict_thread) {
        m_auto_evict_thread->join();
        m_auto_evict_thread = nullptr;
    }
    return 0;
}

int ExpirationCache::add_memory_block(
    const ObjectIdentifier &object_id, const ShmPutOptions &put_options)
{
    std::lock_guard<std::mutex> lock(m_impl->m_cache_mutex);

    RDX_INFO_DEV(nullptr, __func__, "Adding memory block to expiration cache: {}", object_id.to_string());

    // if alive_duration is not set, treat it as no expiration
    // this is handled in DataBlockInfo constructor
    auto data_block_info = DataBlockInfo(object_id, put_options);
    auto &left = m_impl->m_cache.left;
    auto it = left.find(object_id);
    if (it != left.end()) {
        // Update existing entry
        RDX_INFO_DEV(nullptr, __func__, "Updating existing entry: {}", object_id.to_string());
        left.replace_data(it, data_block_info);
    } else {
        // Insert new entry
        RDX_INFO_DEV(nullptr, __func__, "Inserting new entry: {}, with put options: {}",
                     object_id.to_string(), put_options.to_string());
        left.insert(std::make_pair(object_id, data_block_info));
    }

    return 0;
}

int64_t ExpirationCache::evict_expired_memory_blocks(bool force_evict_all)
{
    std::lock_guard<std::mutex> lock(m_impl->m_cache_mutex);

    auto &right = m_impl->m_cache.right;
    auto time_now = std::chrono::system_clock::now();
    if (force_evict_all) {
        time_now = TimePoint_t::max();
    }
    auto range = right.range(bimaps::unbounded, bimaps::_key <= time_now);

    if (range.first == range.second) {
        // no expired blocks found
        return 0;
    } else {
        RDX_INFO_DEV(nullptr, __func__, "Found {} expired blocks", std::distance(range.first, range.second));
    }

    int64_t num_evicted = 0;
    for (auto it = range.first; it != range.second; ++it) {
        auto object_id = it->get_left();
        auto &data_block_info = it->get_right();

        RDX_INFO_DEV(nullptr, __func__, "Processing expired block: {}", object_id.to_string());

        // have callback? call it
        ShmPutOptions::ExpiredAction action = ShmPutOptions::ExpiredAction::DontCare;
        if (data_block_info.put_options.on_expired) {
            RDX_INFO_DEV(nullptr, __func__, "Calling expiration callback for block: {}", object_id.to_string());
            action = data_block_info.put_options.on_expired(object_id, m_client, time_now, data_block_info.put_options);
        }

        // delete the object from shm service
        if (action != ShmPutOptions::ExpiredAction::Keep) {
            RDX_INFO_DEV(nullptr, __func__, "Deleting expired block from shared memory: {}", object_id.to_string());
            m_client->delete_object(object_id);

            // remove from cache
            RDX_INFO_DEV(nullptr, __func__, "Removing expired block from cache: {}", object_id.to_string());
            right.erase(it);
            num_evicted++;
        } else {
            RDX_INFO_DEV(nullptr, __func__, "Keeping expired block: {} as requested", object_id.to_string());
        }
    }

    RDX_INFO_DEV(nullptr, __func__, "Completed eviction, removed {} expired blocks", num_evicted);
    return num_evicted;
}

} // namespace redoxi_works::shared_memory
