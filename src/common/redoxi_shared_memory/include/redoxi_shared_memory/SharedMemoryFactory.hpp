#pragma once

#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <json_struct/json_struct.h>
#include <rclcpp/node.hpp>

namespace redoxi_works
{
namespace shared_memory
{

namespace env_names
{
constexpr const char *RegionKey = "RDX_SHM_REGION_KEY";
constexpr const char *ShmServiceName = "RDX_SHM_SERVICE_NAME";
} // namespace env_names

namespace shm_service_types
{
constexpr const char *Vineyard = "vineyard";
} // namespace shm_service_types

struct SharedMemoryConfig {
    // shm env keys, do not modify them
    std::string _env_region_key = shared_memory::env_names::RegionKey;
    std::string _env_service_name = shared_memory::env_names::ShmServiceName;

    // shm service type, you can set the env service type to this value
    std::string _service_name_vineyard = shared_memory::shm_service_types::Vineyard;

    // if some shm setting is not given, then read them from env variable
    bool read_setting_from_env = true;

    // shm region key, if not given, then read from env variable
    // if given, ignore env variable
    std::string region_key;

    // service name, if not given, then read from env variable
    // if given, ignore env variable
    std::string service_name;

    JS_OBJECT(JS_MEMBER(_env_region_key),
              JS_MEMBER(_env_service_name),
              JS_MEMBER(_service_name_vineyard),
              JS_MEMBER(read_setting_from_env),
              JS_MEMBER(region_key),
              JS_MEMBER(service_name));
};


//! Factory class for creating shared memory clients
class SharedMemoryFactory
{
  public:
    //! Create a shared memory client based on environment variables,
    //! and automatically connect to the shared memory region.
    //! @return a shared memory client, nullptr if failed
    static std::shared_ptr<SharedMemoryClient>
        create_client_by_env();

    //! Create a shared memory client based on the service type
    //! @return a shared memory client, nullptr if failed
    static std::shared_ptr<SharedMemoryClient>
        create_client_by_service_name(const std::string &service_type);

    static std::shared_ptr<SharedMemoryClient>
        create_client_by_config(const SharedMemoryConfig &config);

    //! Get the shared memory service type from environment variables
    static std::string get_shm_service_name_from_env();

    //! Get the shared memory region key from environment variables
    static std::string get_shm_region_key_from_env();

    //! Get the shared memory config from a ros node's parameters
    static SharedMemoryConfig get_shm_config_from_node(const rclcpp::Node *node);
};
} // namespace shared_memory
} // namespace redoxi_works
