#ifndef REDOXI_COMMON_NODES__REDOXI_COMMON_NODES_HPP_
#define REDOXI_COMMON_NODES__REDOXI_COMMON_NODES_HPP_

#include "redoxi_common_nodes/visibility_control.h"
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <json_struct/json_struct.h>
#include <unordered_map>


namespace redoxi_works
{
template <typename InitConfigType, typename RuntimeConfigType>
struct NodeConfigTemplate {
    std::unordered_map<std::string, std::string> declare_params;
    InitConfigType init_config;
    RuntimeConfigType runtime_config;

    JS_OBJECT(
        JS_MEMBER(declare_params),
        JS_MEMBER(init_config),
        JS_MEMBER(runtime_config));
};

template <typename InitConfigType>
struct NodeConfigTemplateInitOnly {
    std::unordered_map<std::string, std::string> declare_params;
    InitConfigType init_config;

    JS_OBJECT(
        JS_MEMBER(declare_params),
        JS_MEMBER(init_config));
};
} // namespace redoxi_works

namespace redoxi_works::port_handlers
{
//! Resource token concept requires copy constructible and copyable
template <typename T>
concept ResourceTokenConcept = requires(T a)
{
    T{};
    requires std::copy_constructible<T>;
    requires std::copyable<T>;
};

struct PullProcessSendHandlerConfig {
    //! Whether to block the input reading
    bool block_input_reading = false;

    //! Whether to block the resource acquisition
    bool block_resource_acquisition = false;
};

struct DummyResourceToken {
};
} // namespace redoxi_works::port_handlers

#endif // REDOXI_COMMON_NODES__REDOXI_COMMON_NODES_HPP_
