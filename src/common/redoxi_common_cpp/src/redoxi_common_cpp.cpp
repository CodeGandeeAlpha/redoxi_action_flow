#include "redoxi_common_cpp/redoxi_common_cpp.hpp"
#include <redoxi_common_cpp/redoxi_options.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>

namespace redoxi_works::options
{
MessageStorageOptions::MessageStorageOptions()
{
    shm_put_options = ShmPutOptions_t();
    shm_put_options->alive_duration = DefaultTimeout;
}

MessageStorageOptions::TimeUnit_t MessageStorageOptions::DefaultTimeout = std::chrono::milliseconds(3000);
} // namespace redoxi_works::options
