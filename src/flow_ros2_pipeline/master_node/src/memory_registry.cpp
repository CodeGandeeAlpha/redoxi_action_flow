#include "psg_common/psg_common.hpp"
#include <master_node/memory_registry.hpp>
#include <master_node/master_node.hpp>

#include <memory>
#include <optional>
#include <vineyard/client/client.h>
#include <rclcpp/logger.hpp>

namespace FlowRos2Pipeline {
    class MemoryRegistryImpl {
    public:
        MemoryRegistryImpl(rclcpp::Node* node) : parent_node(node), logger(node->get_logger()) {}
        virtual ~MemoryRegistryImpl() {}
        std::shared_ptr<vineyard::Client> v6d_client;
        rclcpp::Node* parent_node;
        rclcpp::Logger logger;
    };

    MemoryRegistry::MemoryRegistry(rclcpp::Node* node) {
        m_impl = std::make_shared<MemoryRegistryImpl>(node);

        m_impl->v6d_client = std::make_shared<vineyard::Client>();
    }

    MemoryRegistry::~MemoryRegistry() {
        close_v6d_connection();
    }

    int MemoryRegistry::connect_to_v6d(const std::string& v6d_endpoint) {
        // v6d init
        auto status = m_impl->v6d_client->Connect(v6d_endpoint);
        if (!status.ok()) {
            RCLCPP_ERROR(m_impl->logger, "[MasterNode] Failed to connect to IPCServer: %s", v6d_endpoint.c_str());
            return ReturnCode::ERROR;
        }
        RCLCPP_INFO(m_impl->logger, "[MasterNode] v6d Connected to IPCServer: %s", v6d_endpoint.c_str());
        return ReturnCode::SUCCESS;
    }

    int MemoryRegistry::close_v6d_connection() {
        m_impl->v6d_client->Disconnect();
        auto return_code = remove_all_entries();
        return return_code;
    }

    int MemoryRegistry::add_entry(const MemoryEntry& entry) {
        m_entries[entry.frame_number][entry.name] = entry;
        return ReturnCode::SUCCESS;
    }

    int MemoryRegistry::remove_entry(int frame_number, const std::string& name) {
        m_entries[frame_number].erase(name);
        return ReturnCode::SUCCESS;
    }

    int MemoryRegistry::remove_entries_by_frame(int frame_number) {
        m_entries.erase(frame_number);
        return ReturnCode::SUCCESS;
    }

    int MemoryRegistry::remove_all_entries() {
        m_entries.clear();
        return ReturnCode::SUCCESS;
    }

    std::optional<std::reference_wrapper<const MemoryEntry>> MemoryRegistry::get_entry(int frame_number, const std::string& name) const {
        if (m_entries.find(frame_number) == m_entries.end()) {
            return std::nullopt;
        }
        if (m_entries.at(frame_number).find(name) == m_entries.at(frame_number).end()) {
            return std::nullopt;
        }
        const auto& entry = m_entries.at(frame_number).at(name);
        return std::cref(entry);
    }

    std::optional<std::reference_wrapper<const MemoryRegistry::FrameEntries>> MemoryRegistry::get_entries_by_frame(int frame_number) const {
        auto it = m_entries.find(frame_number);
        // avoid throwing exception
        if (it == m_entries.end()) {
            return std::nullopt;
        }

        return std::cref(it->second);
    }

    const MemoryRegistry::MemoryEntriesByFrame& MemoryRegistry::get_all_entries() const {
        return m_entries;
    }



}