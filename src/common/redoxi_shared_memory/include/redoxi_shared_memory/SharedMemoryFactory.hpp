#pragma once

#include <redoxi_shared_memory/SharedMemoryClient.hpp>

namespace redoxi_works
{
namespace shared_memory
{

namespace env_names
{
constexpr const char *RegionKey = "RDX_SHM_REGION_KEY";
constexpr const char *ShmServiceType = "RDX_SHM_SERVICE_TYPE";
} // namespace env_names

namespace shm_service_types
{
constexpr const char *Vineyard = "vineyard";
} // namespace shm_service_types

//! Factory class for creating shared memory clients
class SharedMemoryFactory
{
  public:
    //! Create a shared memory client based on environment variables,
    //! and automatically connect to the shared memory region.
    //! @return a shared memory client, nullptr if failed
    static std::shared_ptr<SharedMemoryClient>
        create_client_by_env();

    //! Get the shared memory service type from environment variables
    static std::string get_shm_service_type_from_env();

    //! Get the shared memory region key from environment variables
    static std::string get_shm_region_key_from_env();
};
} // namespace shared_memory
} // namespace redoxi_works
