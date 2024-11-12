#pragma once

#include <redoxi_shm_v6d/redoxi_shm_v6d.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
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

    // SharedMemoryClient interface, KeyValueStore is VineyardParams object
    int connect(const std::string &region_key, const KeyValueStore *params = nullptr) override;

    //! Get the region key, empty string if not set yet
    const std::string &get_region_key() const override;

    //! Check if the client is connected
    bool is_connected() const override;

    //! Close the client
    int close() override;

    // SharedMemoryClient interface, DataBlock is VineyardDataBlock object, KeyValueStore is VineyardParams object
    int put_data(ObjectIdentifier *output_object_id,
                 const DataBlock *data_block,
                 const KeyValueStore *metadata = nullptr) override;

    // SharedMemoryClient interface, DataBlock is VineyardDataBlock object, KeyValueStore is VineyardParams object
    int get_data(DataBlock *output_data_block,
                 KeyValueStore *output_metadata,
                 const ObjectIdentifier &identifier) override;

    // SharedMemoryClient interface, KeyValueStore is VineyardParams object
    int delete_object(const ObjectIdentifier &identifier,
                      const KeyValueStore *metadata = nullptr) override;

    // SharedMemoryClient interface
    void set_parent_node(rclcpp::Node *node) override;
    rclcpp::Node *get_parent_node() override;
    const rclcpp::Node *get_parent_node() const override;

    // SharedMemoryClient interface
    std::shared_ptr<DataBlock> create_datablock() const override;
    std::shared_ptr<KeyValueStore> create_kvstore() const override;

    // get the service name of this shared memory service
    const std::string &get_service_name() const override;

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
    rclcpp::Node *m_node = nullptr;
    std::string m_region_key;
    rclcpp::Logger _get_logger() const;
};

} // namespace redoxi_works::shared_memory
