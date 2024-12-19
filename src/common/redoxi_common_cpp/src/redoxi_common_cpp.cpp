#include <cstdlib>
#include <cstdint>
#include "redoxi_common_cpp/redoxi_common_cpp.hpp"
#include <redoxi_common_cpp/redoxi_options.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>

namespace redoxi_works::options
{
MessageStorageOptions::MessageStorageOptions()
{
    shm_put_options = ShmPutOptions_t();
    shm_put_options->alive_duration = DefaultTimeout;

    // get the default alive duration from environment variable
    const char *shm_alive_duration_str = std::getenv(shared_memory::config_keys::env::ShmAliveDuration.data());
    if (shm_alive_duration_str && *shm_alive_duration_str != '\0') {
        try {
            double duration = std::stod(shm_alive_duration_str);
            shm_put_options->alive_duration = std::chrono::duration_cast<TimeUnit_t>(std::chrono::duration<double>(duration));
        } catch (...) {
            // If parsing fails, keep the default alive duration
        }
    }
}

MessageStorageOptions::TimeUnit_t MessageStorageOptions::DefaultTimeout = std::chrono::milliseconds(3000);
} // namespace redoxi_works::options
