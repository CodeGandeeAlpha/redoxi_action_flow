#pragma once

#include <redoxi_common_cpp/visibility_control.h>
#include <optional>
#include <redoxi_shared_memory/SharedMemoryTypes.hpp>

namespace redoxi_works::options
{

// when large chunk of data is being converted to message,
// we can choose how to deal with it
struct MessageStorageOptions {
    using ShmPutOptions_t = shared_memory::ShmPutOptions;
    enum class StorageType {
        Auto,          // auto choose the best way to store the data
        RosSerialized, // serialize the data to a buffer using ros default serializer
        SharedMemory,  // put the data to shared memory
    };

    StorageType storage_type = StorageType::Auto;
    std::optional<ShmPutOptions_t> shm_put_options; // shm put options, only used when storage_type is SharedMemory
};
} // namespace redoxi_works::options
