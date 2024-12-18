#pragma once

#include <redoxi_shm_v6d/redoxi_shm_v6d.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/ExpirationCache.hpp>
#include <vineyard/client/client.h>
#include <vineyard/basic/ds/tensor.h>
namespace redoxi_works::shared_memory
{

class VineyardShmClient : public SharedMemoryClient
{
  public:
    using VineyardTensor_u8 = vineyard::Tensor<uint8_t>;
    struct VineyardDataBlock : public detail::DefaultDataBlock {
        // you must have this to hold the tensor data
        std::shared_ptr<VineyardTensor_u8> tensor;
        friend class VineyardShmClient;
    };
    struct VineyardParams : public detail::DefaultKeyValueStore {
    };

  public:
    VineyardShmClient();
    virtual ~VineyardShmClient();

    using SharedMemoryClient::get_data;

    // SharedMemoryClient interface, KeyValueStore is VineyardParams object
    int connect(const SharedMemoryConfig &config,
                std::shared_ptr<KeyValueStore> additional_params = nullptr) override;

    //! Check if the client is connected
    bool is_connected() const override;

    //! Get the connection config of the shared memory client, provided during the connect call.
    const SharedMemoryConfig &get_shm_config() const override;

    //! Get the connection params of the shared memory client, provided during the connect call.
    std::shared_ptr<const KeyValueStore> get_connection_params() const override;

    //! Close the client
    int close() override;

    // SharedMemoryClient interface, DataBlock is VineyardDataBlock object, KeyValueStore is VineyardParams object
    int put_data(ObjectIdentifier *output_object_id,
                 const DataBlock *data_block,
                 const KeyValueStore *metadata = nullptr,
                 std::optional<ShmPutOptions> put_options = std::nullopt) override;

    // SharedMemoryClient interface, DataBlock is VineyardDataBlock object, KeyValueStore is VineyardParams object
    int get_data(DataBlock *output_data_block,
                 KeyValueStore *output_metadata,
                 const ObjectIdentifier &identifier) override;

    // SharedMemoryClient interface, KeyValueStore is VineyardParams object
    int delete_object(const ObjectIdentifier &identifier,
                      const KeyValueStore *metadata = nullptr) override;

    // SharedMemoryClient interface
    std::shared_ptr<DataBlock> create_datablock() const override;
    std::shared_ptr<KeyValueStore> create_kvstore() const override;

    // eviction control
    void set_default_alive_duration(std::optional<TimeUnit_t> default_alive_duration) override;
    void set_max_alive_duration(std::optional<TimeUnit_t> max_alive_duration) override;
    std::optional<TimeUnit_t> get_default_alive_duration() const override;
    std::optional<TimeUnit_t> get_max_alive_duration() const override;

    void _evict_expired_data_blocks(bool force = false) override;
    void _set_auto_evict_enabled(bool enable) override;
    bool _get_auto_evict_enabled() const override;
    // --- Vineyard specific methods ---
  public:
    //! Get the underlying vineyard client
    std::shared_ptr<vineyard::Client> get_client() const;

    //! Put cv::Mat into vineyard
    int put_data(vineyard::ObjectID *output_object_id, const cv::Mat &mat);

    //! put raw data to vineyard
    int put_data(vineyard::ObjectID *output_object_id, const uint8_t *data, size_t size);

    //! Get cv::Mat from vineyard
    std::shared_ptr<VineyardTensor_u8> get_data(vineyard::ObjectID object_id);

  private:
    std::shared_ptr<vineyard::Client> m_client;

    // connection config, common to all shm clients
    SharedMemoryConfig m_config;

    // specific params for this shm client
    std::shared_ptr<VineyardParams> m_additional_params;

    // Expiration cache
    ExpirationCache m_expiration_cache;
};

} // namespace redoxi_works::shared_memory
