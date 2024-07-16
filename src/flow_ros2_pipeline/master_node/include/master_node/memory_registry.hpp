#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <map>
#include <functional>
#include <optional>
#include <psg_common/psg_common.hpp>
#include <rclcpp/node.hpp>

namespace FlowRos2Pipeline {
    class MemoryRegistryImpl;

    class MemoryEntry
    {
    public:
        int frame_number = -1;

        // refers to the memory taken by the object
        std::uint64_t v6d_object_id = 0;

        // name of this object, must be unique in a frame
        std::string name;
    };

    //record all allocated v6d entries, for garbage collection
    class MemoryRegistry
    {
    public:
        MemoryRegistry(rclcpp::Node* node);
        using FrameEntries = std::map<std::string, MemoryEntry>;
        using MemoryEntriesByFrame = std::map<int, FrameEntries>;

        ~MemoryRegistry();

    // management api
    public:
        int connect_to_v6d(const std::string& v6d_endpoint);
        int close_v6d_connection();

    // memory access api
    public:
        int add_entry(const MemoryEntry& entry);
        int remove_entry(int frame_number, const std::string& name);
        int remove_entries_by_frame(int frame_number);
        int remove_all_entries();

        std::optional<std::reference_wrapper<const MemoryEntry>> get_entry(int frame_number, const std::string& name) const;
        std::optional<std::reference_wrapper<const FrameEntries>> get_entries_by_frame(int frame_number) const;
        const MemoryEntriesByFrame& get_all_entries() const;

    private:
        MemoryEntriesByFrame m_entries;
        std::shared_ptr<MemoryRegistryImpl> m_impl;
    };
}