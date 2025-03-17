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
    std::recursive_mutex m_cache_mutex;
};

ExpirationCache::ExpirationCache()
    : m_impl(std::make_shared<Impl>())
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

void ExpirationCache::set_on_evict_callback(OnEvictCallback_t callback)
{
    m_on_evict_callback = callback;
}

int ExpirationCache::add_memory_block(
    const ObjectIdentifier &object_id, const ShmPutOptions &put_options)
{
    RDX_INFO_DEV(nullptr, __func__, "Adding memory block to expiration cache: {}, put options: {}",
                 object_id.to_string(), put_options.to_string());
    std::scoped_lock lock(m_impl->m_cache_mutex);

    RDX_INFO_DEV(nullptr, __func__, "{}", "cache mutex locked");

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

// FIXME: code duplication with evict_expired_memory_blocks, should be refactored
int ExpirationCache::remove_memory_block(const ObjectIdentifier &object_id, bool treat_it_as_expired)
{
    RDX_INFO_DEV(nullptr, __func__, "removing memory block from cache, object id={}, as expired={}",
                 object_id.to_string(), treat_it_as_expired);
    std::scoped_lock lock(m_impl->m_cache_mutex);
    RDX_INFO_DEV(nullptr, __func__, "{}", "cache mutex locked");

    // look up the object id in the cache
    auto &left_map = m_impl->m_cache.left;
    auto it = left_map.find(object_id);
    if (it == left_map.end()) {
        // not found
        return -1;
    }

    // got it, now remove it
    auto &data_block_info = it->get_right();
    RDX_INFO_DEV(nullptr, __func__, "Removing memory block from cache: {}", object_id.to_string());

    if (treat_it_as_expired) {
        // have callback? call it
        ShmPutOptions::ExpiredAction action = ShmPutOptions::ExpiredAction::DontCare;
        if (data_block_info.put_options.on_expired) {
            RDX_INFO_DEV(nullptr, __func__, "Calling per-block expiration callback for block: {}", object_id.to_string());
            action = data_block_info.put_options.on_expired(
                object_id, std::chrono::system_clock::now(), data_block_info.put_options);
        }

        // user agreed to evict, do it
        if (action != ShmPutOptions::ExpiredAction::Keep) {
            // remove from cache
            RDX_INFO_DEV(nullptr, __func__, "Removing expired block from cache: {}", object_id.to_string());
            m_impl->m_cache.left.erase(it);

            // call the callback
            if (m_on_evict_callback) {
                RDX_INFO_DEV(nullptr, __func__, "Calling universal eviction callback for block: {}", object_id.to_string());
                int ret = m_on_evict_callback(object_id, *this);
                if (ret != 0) {
                    RDX_WARN_DEV(nullptr, __func__, "Universal eviction callback failed for block: {}, continue evicting",
                                 object_id.to_string());
                } else {
                    RDX_INFO_DEV(nullptr, __func__, "Universal eviction callback succeeded for block: {}", object_id.to_string());
                }
            }
        } else {
            RDX_INFO_DEV(nullptr, __func__, "Keeping expired block: {} as requested", object_id.to_string());
        }
        return 0;
    } else {
        // not treated as expired, just remove it
        m_impl->m_cache.left.erase(it);
        return 0;
    }
    RDX_INFO_DEV(nullptr, __func__, "{}", "cache mutex unlocked, removed memory block");
}

int64_t ExpirationCache::evict_expired_memory_blocks(bool force_evict_all)
{
    RDX_INFO_DEV(nullptr, __func__, "{}", "evicting expired memory blocks");
    std::scoped_lock lock(m_impl->m_cache_mutex);
    RDX_INFO_DEV(nullptr, __func__, "{}", "cache mutex locked");

    auto &right = m_impl->m_cache.right;
    auto time_now = std::chrono::system_clock::now();
    if (force_evict_all) {
        time_now = TimePoint_t::max();
    }
    auto range = right.range(bimaps::unbounded, bimaps::_key <= time_now);

    if (range.first == range.second) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "No expired blocks found, releasing cache mutex");
        // no expired blocks found
        return 0;
    } else {
        RDX_INFO_DEV(nullptr, __func__, "Found {} expired blocks", std::distance(range.first, range.second));
    }

    std::vector<ObjectIdentifier> object_ids_to_evict;
    for (auto it = range.first; it != range.second; ++it) {
        auto object_id = it->get_left();
        auto &data_block_info = it->get_right();

        RDX_INFO_DEV(nullptr, __func__, "Processing expired block: {}", object_id.to_string());

        // have callback? call it
        ShmPutOptions::ExpiredAction action = ShmPutOptions::ExpiredAction::DontCare;
        if (data_block_info.put_options.on_expired) {
            RDX_INFO_DEV(nullptr, __func__, "Calling expiration callback for block: {}", object_id.to_string());
            action = data_block_info.put_options.on_expired(object_id, time_now, data_block_info.put_options);
        }

        // delete the object from shm service
        if (action != ShmPutOptions::ExpiredAction::Keep) {
            // mark for removal from cache
            RDX_INFO_DEV(nullptr, __func__, "Removing expired block from cache: {}", object_id.to_string());
            object_ids_to_evict.push_back(object_id);

            // call the callback
            if (m_on_evict_callback) {
                RDX_INFO_DEV(nullptr, __func__, "Calling evict callback for block: {}", object_id.to_string());
                int ret = m_on_evict_callback(object_id, *this);
                if (ret != 0) {
                    RDX_WARN_DEV(nullptr, __func__, "Evict callback failed for block: {}, continue evicting",
                                 object_id.to_string());
                } else {
                    RDX_INFO_DEV(nullptr, __func__, "Evict callback succeeded for block: {}", object_id.to_string());
                }
            }
        } else {
            RDX_INFO_DEV(nullptr, __func__, "Keeping expired block: {} as requested", object_id.to_string());
        }
    }

    // remove the expired blocks from cache
    for (const auto &object_id : object_ids_to_evict) {
        m_impl->m_cache.left.erase(object_id);
    }

    RDX_INFO_DEV(nullptr, __func__, "Completed eviction, removed {} expired blocks", object_ids_to_evict.size());
    return object_ids_to_evict.size();
}

} // namespace redoxi_works::shared_memory
