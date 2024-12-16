#pragma once

#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <json_struct/json_struct.h>
#include <rclcpp/node.hpp>

namespace redoxi_works::shared_memory
{

namespace config_keys::node
{
constexpr std::string_view ServiceType = "shm_service_type";
constexpr std::string_view RegionKey = "shm_region_key";
} // namespace config_keys::node

namespace config_keys::env
{
constexpr std::string_view ServiceType = "RDX_SHM_SERVICE_TYPE";
constexpr std::string_view RegionKey = "RDX_SHM_REGION_KEY";
} // namespace config_keys::env

namespace config_values::service_types
{
constexpr std::string_view Vineyard = "vineyard";
} // namespace config_values::service_types

struct SharedMemoryConfig {
    // shm configuration keys and values, just for documentations, do not modify them
    std::string _env_config_service_type = config_keys::env::ServiceType.data();
    std::string _env_config_region_key = config_keys::env::RegionKey.data();
    std::string _node_config_service_type = config_keys::node::ServiceType.data();
    std::string _node_config_region_key = config_keys::node::RegionKey.data();

    // shm service type enum values, just for documentations, do not modify them
    std::string _service_type_vineyard = config_values::service_types::Vineyard.data();

    // shm region key, if not given, then read from env variable
    // if given, ignore env variable
    std::string region_key;

    // service name, if not given, then read from env variable
    // if given, ignore env variable
    std::string service_type;

    //! Comparison operators for use in std::map
    bool operator<(const SharedMemoryConfig &other) const
    {
        return std::tie(service_type, region_key) < std::tie(other.service_type, other.region_key);
    }

    bool operator==(const SharedMemoryConfig &other) const
    {
        return std::tie(service_type, region_key) == std::tie(other.service_type, other.region_key);
    }

    bool operator!=(const SharedMemoryConfig &other) const
    {
        return !(*this == other);
    }

    bool is_valid() const
    {
        return !service_type.empty() && !region_key.empty();
    }

    JS_OBJECT(JS_MEMBER(_env_config_region_key),
              JS_MEMBER(_env_config_service_name),
              JS_MEMBER(_node_config_region_key),
              JS_MEMBER(_node_config_service_name),
              JS_MEMBER(_service_type_vineyard),
              JS_MEMBER(region_key),
              JS_MEMBER(service_type));
};


//! Factory class for creating shared memory clients based on different configurations
class SharedMemoryFactory
{
  public:
    //! Create a shared memory client based on the service type
    //! @return an uninitialized shared memory client, you need to connect it to the shared memory region before use, nullptr if failed
    static std::shared_ptr<SharedMemoryClient>
        create_client_by_service_type(const std::string &service_type);

    //! Create a shared memory client based on the config
    //! @return a connected shared memory client, ready to use, nullptr if failed
    static std::shared_ptr<SharedMemoryClient>
        create_client_by_config(const SharedMemoryConfig &config);

    //! Get the shared memory config from a ros node's parameters
    static SharedMemoryConfig get_shm_config_from_node(const rclcpp::Node *node);

    //! Get the shared memory config from env variables
    static SharedMemoryConfig get_shm_config_from_env();

  private:
    //! Get the shared memory service type from environment variables
    static std::string get_shm_service_name_from_env();

    //! Get the shared memory region key from environment variables
    static std::string get_shm_region_key_from_env();
};
} // namespace redoxi_works::shared_memory
