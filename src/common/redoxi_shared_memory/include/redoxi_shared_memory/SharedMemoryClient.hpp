#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <redoxi_shared_memory/SharedMemoryTypes.hpp>


namespace redoxi_works::shared_memory
{

class SharedMemoryClient
{
  public:
    SharedMemoryClient() = default;
    virtual ~SharedMemoryClient() = default;

    virtual int connect(const SharedMemoryConfig &config,
                        std::shared_ptr<KeyValueStore> additional_params = nullptr) = 0;

    /**
     * @brief Get the config of the shared memory client, provided during the connect call.
     *
     * @return The config of the shared memory client.
     */
    virtual const SharedMemoryConfig &get_connection_config() const = 0;

    /**
     * @brief Get the connection params of the shared memory client, provided during the connect call.
     *
     * @return The connection params of the shared memory client.
     */
    virtual std::shared_ptr<const KeyValueStore> get_connection_params() const = 0;

    /**
     * @brief Put data into the shared memory region.
     *
     * Stores a data block in the shared memory region and assigns an identifier to it.
     *
     * @param output_object_id The identifier for the stored data block.
     * @param data_block The data block to be stored.
     * @param metadata Optional metadata associated with the data block.
     * @param expiration_config Optional expiration config for the data block, if nullptr, then use the default expiration config
     * @return 0 if the operation is successful, -1 if it fails.
     */
    virtual int put_data(ObjectIdentifier *output_object_id,
                         const DataBlock *data_block,
                         const KeyValueStore *metadata = nullptr,
                         const MemoryBlockExpirationConfig *expiration_config = nullptr) = 0;

    /**
     * @brief Retrieve data from the shared memory region.
     *
     * Fetches a data block and its metadata from the shared memory region using the specified identifier.
     *
     * @param output_data_block The data block to be retrieved. If nullptr, data will not be returned.
     * @param output_metadata The metadata to be retrieved. If nullptr, metadata will not be returned.
     * @param identifier The identifier of the data block to be fetched.
     * @return 0 if the retrieval is successful, -1 if it fails.
     */
    virtual int get_data(DataBlock *output_data_block,
                         KeyValueStore *output_metadata,
                         const ObjectIdentifier &identifier) = 0;

    /**
     * @brief Get a data block from the shared memory region, by object identifier only.
     *
     * @param identifier The identifier of the data block to be fetched.
     * @return A shared pointer to the data block, nullptr if failed.
     */
    virtual std::shared_ptr<DataBlock> get_data(const ObjectIdentifier &identifier)
    {
        auto datablock = create_datablock();
        auto ret = get_data(datablock.get(), nullptr, identifier);
        if (ret != 0) {
            return nullptr;
        }
        return datablock;
    }

    /**
     * @brief Check the connection status to the shared memory region.
     *
     * Determines whether the client is currently connected to the shared memory region.
     *
     * @return true if the client is connected, false otherwise.
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Release a shared memory object.
     *
     * Deletes a specified object from the shared memory region using its identifier.
     *
     * @param identifier The identifier of the object to be released. Either the key or the id must be set.
     * @param metadata Optional metadata associated with the object, usually not required.
     * @return 0 if the deletion is successful, -1 if it fails.
     */
    virtual int delete_object(const ObjectIdentifier &identifier,
                              const KeyValueStore *metadata = nullptr) = 0;

    /**
     * @brief Close the connection to the shared memory region.
     *
     * Terminates the connection to the shared memory region.
     *
     * @return 0 if the closure is successful, -1 if it fails.
     */
    virtual int close() = 0;

    /**
     * @brief Create a local data block, which is used for uploading local data to shared memory,
     *        or receiving data from shared memory.
     *
     * @return A shared pointer to the data block.
     */
    virtual std::shared_ptr<DataBlock> create_datablock() const = 0;

    /**
     * @brief Create a local key-value store, which is used for uploading local metadata to shared memory,
     *        or receiving metadata from shared memory.
     *
     * @return A shared pointer to the key-value store.
     */
    virtual std::shared_ptr<KeyValueStore> create_kvstore() const = 0;
};

} // namespace redoxi_works::shared_memory
