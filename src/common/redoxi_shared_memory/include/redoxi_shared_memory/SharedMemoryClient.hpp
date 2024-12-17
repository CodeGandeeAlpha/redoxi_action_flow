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
    /**
     * @brief Connect to a shared memory region.
     *
     * Establishes a connection to a specified shared memory region using the provided key.
     *
     * @param region_key The unique key identifying the shared memory region.
     * @param params Optional parameters for connection, can be user-defined.
     * @return 0 if the connection is successful, -1 if it fails.
     */
    virtual int connect(const std::string &region_key, const KeyValueStore *params = nullptr) = 0;

    // /**
    //  * @brief Set the expiration config for the shared memory client, which will affect all the data blocks put into the shared memory region.
    //  *
    //  * @param expiration_config The expiration config to be set. If nullptr, it will clear the expiration config.
    //  */
    // virtual void set_expiration_config(const MemoryBlockExpirationConfig *expiration_config) = 0;

    // /**
    //  * @brief Get the expiration config for the shared memory client, which will affect all the data blocks put into the shared memory region.
    //  *
    //  * @return The expiration config, nullptr if not set.
    //  */
    // virtual const MemoryBlockExpirationConfig *get_expiration_config() const = 0;

    /**
     * @brief Get the service name of the shared memory region.
     */
    virtual std::string get_service_type() const = 0;

    //! Get the region key, empty string if not connected
    virtual std::string get_region_key() const = 0;

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
     * @brief Set the parent node for the shared memory client, mainly used for logging.
     *
     * @param node The parent node. If nullptr, no logging will be done.
     */
    virtual void set_parent_node(rclcpp::Node *node) = 0;

    /**
     * @brief Get the parent node for the shared memory client, mainly used for logging.
     *
     * @return The parent node.
     */
    virtual rclcpp::Node *get_parent_node() = 0;

    /**
     * @brief Get the parent node for the shared memory client, mainly used for logging.
     *
     * @return The parent node.
     */
    virtual const rclcpp::Node *get_parent_node() const = 0;

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
