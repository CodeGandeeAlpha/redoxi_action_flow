#include <redoxi_video_reader/base/ShmVideoReaderBase.hpp>
#include <typeinfo>

namespace redoxi_works
{
namespace video_reader
{
int ShmVideoReaderBase::init(std::shared_ptr<BaseInitConfig_t> config,
                             std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
{
    // check data type
    auto _runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(runtime_config);
    if (!_runtime_config) {
        RDX_RAISE_ERROR("[f={}()] Invalid runtime config type, expect {}, got {}", __func__,
                        typeid(RuntimeConfig_t).name(),
                        typeid(runtime_config.get()).name());
        return -1;
    }
    return RedoxiVideoReaderBase::init(config, runtime_config);
}

int ShmVideoReaderBase::update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
{
    auto _runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(runtime_config);
    if (!_runtime_config) {
        RDX_RAISE_ERROR("[f={}()] Invalid runtime config type, expect {}, got {}",
                        __func__, typeid(RuntimeConfig_t).name(),
                        typeid(runtime_config.get()).name());
        return -1;
    }
    return RedoxiVideoReaderBase::update_runtime_config(runtime_config);
}

int ShmVideoReaderBase::_open()
{
    // if base class open failed, return error
    if (RedoxiVideoReaderBase::_open() != 0) {
        return -1;
    }

    auto config = _get_runtime_config();

    // create shm client
    auto shm_client = _create_shm_client(config->shm_config);
    if (!shm_client) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to create shm client");
        return -1;
    }
    m_shm_client = shm_client;
    return 0;
}

int ShmVideoReaderBase::_close()
{
    // if base class close failed, return error
    if (RedoxiVideoReaderBase::_close() != 0) {
        return -1;
    }

    if (m_shm_client) {
        m_shm_client->close();
        m_shm_client.reset();
    }
    return 0;
}

std::shared_ptr<ShmVideoReaderBase::RuntimeConfig_t> ShmVideoReaderBase::_get_runtime_config() const
{
    // DO NOT call this in init() or update_runtime_config()
    // it depends on the initialized runtime config

    // if runtime config is not initialized, return nullptr
    // otherwise, it must be a RuntimeConfig_t
    if (!m_runtime_config) {
        return nullptr;
    } else {
        auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
        if (!config) {
            RDX_RAISE_ERROR("[f={}()] Invalid runtime config type, expect {}, got {}", __func__,
                            typeid(RuntimeConfig_t).name(),
                            typeid(m_runtime_config.get()).name());
        }
        return config;
    }
}

int ShmVideoReaderBase::_on_delivery_task_begin(TargetData_t &target_data,
                                                const DeliveryRequest_t &request)
{
    return 0;
}

} // namespace video_reader
} // namespace redoxi_works