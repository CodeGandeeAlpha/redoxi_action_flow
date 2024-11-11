#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <json_struct/json_struct.h>
namespace redoxi_works
{
namespace video_reader
{

namespace shm_video_reader_base
{
struct InitConfig : public RedoxiVideoReaderBase::InitConfig_t {
    // shm env keys, do not modify them
    std::string _shm_env_region_key = shared_memory::env_names::RegionKey;
    std::string _shm_env_service_type = shared_memory::env_names::ShmServiceType;

    // shm service type, you can set the env service type to this value
    std::string _shm_service_type_vineyard = shared_memory::shm_service_types::Vineyard;

    // if some shm setting is not given, then read them from env variable
    bool shm_read_setting_from_env = true;

    // shm region key, if not given, then read from env variable
    // if given, ignore env variable
    std::string shm_region_key;

    // service type, if not given, then read from env variable
    // if given, ignore env variable
    std::string shm_service_type;

    JS_OBJECT_WITH_SUPER(JS_SUPER(RedoxiVideoReaderBase::InitConfig_t),
                         JS_MEMBER(_shm_env_region_key),
                         JS_MEMBER(_shm_env_service_type),
                         JS_MEMBER(_shm_service_type_vineyard),
                         JS_MEMBER(shm_read_setting_from_env),
                         JS_MEMBER(shm_region_key),
                         JS_MEMBER(shm_service_type));
};

} // namespace shm_video_reader_base

//! Video reader that sends data via shared memory
class ShmVideoReaderBase : public RedoxiVideoReaderBase
{
  public:
    explicit ShmVideoReaderBase(const std::string &name,
                                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ShmVideoReaderBase() override = default;

  protected:
    std::shared_ptr<shared_memory::SharedMemoryClient> _create_shm_client() const;

  protected:
    std::shared_ptr<shared_memory::SharedMemoryClient> m_shm_client;
};

} // namespace video_reader
} // namespace redoxi_works
