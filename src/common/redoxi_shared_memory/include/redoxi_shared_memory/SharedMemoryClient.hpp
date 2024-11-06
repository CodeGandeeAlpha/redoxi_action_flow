#pragma once

#include <string>
#include <redoxi_shared_memory/redoxi_shared_memory.hpp>


namespace redoxi_works
{
namespace shared_memory
{

class SharedMemoryClient
{
  public:
  public:
    SharedMemoryClient() = default;
    virtual ~SharedMemoryClient() = default;
    /**
     * @brief Connect to a shared memory region
     * @param region_key The key of the shared memory region
     * @param context The additional user-defined context of the shared memory region
     * @return 0 if success, -1 if failed
     */
    virtual int connect(const std::string &region_key, void *context = nullptr) = 0;

    /**
     * @brief Put data to the shared memory region
     * @param output_object_id The output object identifier
     * @param data_block The data block to put
     * @param metadata The metadata of the data block, optional
     * @return 0 if success, -1 if failed
     */
    virtual int put_data(ObjectIdentifier *output_object_id,
                         const DataBlock *data_block,
                         const Metadata *metadata = nullptr) = 0;

    /**
     * @brief Get data from the shared memory region
     * @param output_data_block The output data block, if nullptr, the data will not be returned
     * @param output_metadata The output metadata, if nullptr, the metadata will not be returned
     * @param identifier The identifier of the data block
     * @return 0 if success, -1 if failed
     */
    virtual int get_data(DataBlock *output_data_block,
                         Metadata *output_metadata,
                         const ObjectIdentifier &identifier) = 0;

    /**
     * @brief Check if the shared memory client is connected to the shared memory region
     * @return true if connected, false otherwise
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Release a shared memory object
     * @param identifier The identifier of the object to release, either the key or the id must be set
     * @param metadata The metadata of the object, optional
     * @return 0 if success, -1 if failed
     */
    virtual int delete_object(const ObjectIdentifier &identifier,
                              const Metadata *metadata = nullptr) = 0;

    /**
     * @brief Close the connection to the shared memory region
     * @return 0 if success, -1 if failed
     */
    virtual int close() = 0;
};

} // namespace shared_memory

} // namespace redoxi_works
