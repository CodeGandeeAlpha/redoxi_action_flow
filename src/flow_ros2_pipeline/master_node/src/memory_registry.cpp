#include <cstdint>
#include <master_node/master_node.hpp>
#include <master_node/memory_registry.hpp>
#include <psg_common/psg_common.hpp>

#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <vineyard/client/client.h>

namespace FlowRos2Pipeline
{
class MemoryRegistryImpl
{
  public:
    MemoryRegistryImpl(rclcpp::Node *node)
        : parent_node(node), logger(node->get_logger())
    {
    }
    virtual ~MemoryRegistryImpl()
    {
    }
    std::shared_ptr<vineyard::Client> v6d_client;
    rclcpp::Node *parent_node;
    rclcpp::Logger logger;
};

MemoryRegistry::MemoryRegistry(rclcpp::Node *node)
{
    m_impl = std::make_shared<MemoryRegistryImpl>(node);

    m_impl->v6d_client = std::make_shared<vineyard::Client>();
}

MemoryRegistry::~MemoryRegistry()
{
    remove_all_entries();
    close_v6d_connection();
}

int MemoryRegistry::connect_to_v6d(const std::string &v6d_endpoint)
{
    // v6d init
    auto status = m_impl->v6d_client->Connect(v6d_endpoint);
    if (!status.ok()) {
        RCLCPP_ERROR(m_impl->logger, "[MemoryRegistry] Failed to connect to IPCServer: %s", v6d_endpoint.c_str());
        return ReturnCode::ERROR;
    }
    // RCLCPP_INFO(m_impl->logger, "[MemoryRegistry] v6d Connected to IPCServer: %s", v6d_endpoint.c_str());
    return ReturnCode::SUCCESS;
}

int MemoryRegistry::close_v6d_connection()
{
    m_impl->v6d_client->Disconnect();
    return ReturnCode::SUCCESS;
}

int MemoryRegistry::add_entry(const MemoryEntry &entry)
{
    m_entries[entry.frame_number][entry.name] = entry;
    return ReturnCode::SUCCESS;
}

int MemoryRegistry::remove_entry(int frame_number, const std::string &name)
{
    if (m_entries.find(frame_number) == m_entries.end()) {
        throw std::runtime_error("[MemoryRegistry] frame_number " + std::to_string(frame_number) + " not found");
        return ReturnCode::ERROR;
    }

    if (m_entries[frame_number].find(name) == m_entries[frame_number].end()) {
        throw std::runtime_error("[MemoryRegistry] name " + name + " not found");
        return ReturnCode::ERROR;
    }

    auto return_status = m_impl->v6d_client->DelData(m_entries[frame_number][name].v6d_object_id);
    if (!return_status.ok()) {
        RCLCPP_WARN(m_impl->logger, "[MemoryRegistry] Failed to delete v6d data, object id is %lu", m_entries[frame_number][name].v6d_object_id);
    }

    m_entries[frame_number].erase(name);
    return ReturnCode::SUCCESS;
}

int MemoryRegistry::remove_entries_by_frame(int frame_number)
{
    if (m_entries.find(frame_number) == m_entries.end()) {
        throw std::runtime_error("[MemoryRegistry] frame_number " + std::to_string(frame_number) + " not found");
        return ReturnCode::ERROR;
    }

    std::vector<std::uint64_t> vec_ids;
    for (const auto &entry : m_entries[frame_number]) {
        vec_ids.push_back(entry.second.v6d_object_id);
    }
    auto return_status = m_impl->v6d_client->DelData(vec_ids);
    if (!return_status.ok()) {
        RCLCPP_WARN(m_impl->logger, "[MemoryRegistry] Failed to delete v6d data in frame %d", frame_number);
    }

    m_entries.erase(frame_number);
    return ReturnCode::SUCCESS;
}

int MemoryRegistry::remove_all_entries()
{
    std::vector<std::uint64_t> vec_ids;
    for (const auto &frame_entries : m_entries) {
        for (const auto &entry : frame_entries.second) {
            vec_ids.push_back(entry.second.v6d_object_id);
        }
    }

    auto return_status = m_impl->v6d_client->DelData(vec_ids);

    if (!return_status.ok()) {
        RCLCPP_WARN(m_impl->logger, "[MemoryRegistry] Failed to delete v6d data");
    }

    m_entries.clear();
    return ReturnCode::SUCCESS;
}

const MemoryEntry *MemoryRegistry::get_entry(int frame_number, const std::string &name) const
{
    if (m_entries.find(frame_number) == m_entries.end()) {
        return nullptr;
    }
    if (m_entries.at(frame_number).find(name) == m_entries.at(frame_number).end()) {
        return nullptr;
    }
    return &m_entries.at(frame_number).at(name);
}

const MemoryRegistry::FrameEntries *MemoryRegistry::get_entries_by_frame(int frame_number) const
{
    auto it = m_entries.find(frame_number);
    // avoid throwing exception
    if (it == m_entries.end()) {
        return nullptr;
    }

    return &it->second;
}

const MemoryRegistry::MemoryEntriesByFrame &MemoryRegistry::get_all_entries() const
{
    return m_entries;
}

} // namespace FlowRos2Pipeline