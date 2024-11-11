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
struct SharedMemoryConfig {
    // shm env keys, do not modify them
    std::string _env_region_key = shared_memory::env_names::RegionKey;
    std::string _env_service_type = shared_memory::env_names::ShmServiceType;

    // shm service type, you can set the env service type to this value
    std::string _service_type_vineyard = shared_memory::shm_service_types::Vineyard;

    // if some shm setting is not given, then read them from env variable
    bool read_setting_from_env = true;

    // shm region key, if not given, then read from env variable
    // if given, ignore env variable
    std::string region_key;

    // service type, if not given, then read from env variable
    // if given, ignore env variable
    std::string service_type;

    JS_OBJECT(JS_MEMBER(_env_region_key),
              JS_MEMBER(_env_service_type),
              JS_MEMBER(_service_type_vineyard),
              JS_MEMBER(read_setting_from_env),
              JS_MEMBER(region_key),
              JS_MEMBER(service_type));
};

struct RuntimeConfig : public RedoxiVideoReaderBase::RuntimeConfig_t {
    SharedMemoryConfig shm_config;
    JS_OBJECT_WITH_SUPER(JS_SUPER(RedoxiVideoReaderBase::RuntimeConfig_t),
                         JS_MEMBER(shm_config));
};

} // namespace shm_video_reader_base

//! Video reader that sends data via shared memory
class ShmVideoReaderBase : public RedoxiVideoReaderBase
{
  public:
    explicit ShmVideoReaderBase(const std::string &name,
                                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ShmVideoReaderBase() override = default;

    using RuntimeConfig_t = shm_video_reader_base::RuntimeConfig;
    using SharedMemoryConfig_t = shm_video_reader_base::SharedMemoryConfig;
    using BaseInitConfig_t = RedoxiVideoReaderBase::InitConfig_t;
    using BaseRuntimeConfig_t = RedoxiVideoReaderBase::RuntimeConfig_t;

    // public:
    int init(std::shared_ptr<BaseInitConfig_t> config,
             std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

    int update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

  protected:
    int _open() override;
    int _close() override;

    int _on_delivery_task_begin(TargetData_t &target_data,
                                const DeliveryRequest_t &request) override;

    int _on_delivery_task_finish(TargetData_t &target_data,
                                 const DeliveryRequest_t &request,
                                 const DeliveryResult_t &result) override;

    std::shared_ptr<shared_memory::SharedMemoryClient> _create_shm_client(
        const SharedMemoryConfig_t &shm_config) const;

  protected:
    std::shared_ptr<shared_memory::SharedMemoryClient> m_shm_client;

  private:
    std::shared_ptr<RuntimeConfig_t> _get_runtime_config() const;
};

} // namespace video_reader
} // namespace redoxi_works
