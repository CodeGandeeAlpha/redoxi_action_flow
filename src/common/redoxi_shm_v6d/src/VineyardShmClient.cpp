#include <redoxi_shm_v6d/VineyardShmClient.hpp>
#include <redoxi_shm_v6d/v6d_helpers.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>


namespace redoxi_works::shared_memory
{

VineyardShmClient::VineyardShmClient()
{
}

VineyardShmClient::~VineyardShmClient()
{
}

int VineyardShmClient::connect(const std::string &region_key,
                               const KeyValueStore *)
{
    try {
        m_client = create_v6d_client(region_key);
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during connection
        return -1;
    }
}

std::shared_ptr<vineyard::Client> VineyardShmClient::get_client() const
{
    return m_client;
}

int VineyardShmClient::close()
{
    if (!m_client || !m_client->Connected()) {
        return 0;
    }

    try {
        m_client->CloseSession();
        return 0;
    } catch (const std::exception &e) {
        //! Handle any exceptions that might occur during the process
        return -1;
    }
}

int VineyardShmClient::put_data(vineyard::ObjectID *object_id, const cv::Mat &mat)
{
    try {
        int height = mat.rows;
        int width = mat.cols;
        int elem_size = mat.elemSize();

        vineyard::TensorBuilder<uint8_t> builder(*m_client, {height, width, elem_size});
        auto tensor_data = builder.data();

        std::memcpy(tensor_data, mat.data, height * width * elem_size);

        auto sealed = std::dynamic_pointer_cast<vineyard::Tensor<uint8_t>>(builder.Seal(*m_client));
        VINEYARD_CHECK_OK(m_client->Persist(sealed->id()));

        //! Set the object id if it is not null
        if (object_id) {
            *object_id = sealed->id();
        }
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        return -1;
    }
}

int VineyardShmClient::put_data(vineyard::ObjectID *output_object_id,
                                const uint8_t *data, size_t size)
{
    try {
        vineyard::TensorBuilder<uint8_t> builder(*m_client, {int64_t(size)});
        auto tensor_data = builder.data();

        std::memcpy(tensor_data, data, size);

        auto sealed = std::dynamic_pointer_cast<vineyard::Tensor<uint8_t>>(builder.Seal(*m_client));
        VINEYARD_CHECK_OK(m_client->Persist(sealed->id()));

        //! Set the object id if it is not null
        if (output_object_id) {
            *output_object_id = sealed->id();
        }
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        return -1;
    }
}

std::shared_ptr<VineyardShmClient::VineyardTensor_u8>
    VineyardShmClient::get_data(vineyard::ObjectID object_id)
{
    try {
        auto tensor = get_tensor_by_v6d_id<uint8_t>(object_id, m_client.get());
        return tensor;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        return nullptr;
    }
}

bool VineyardShmClient::is_connected() const
{
    return m_client != nullptr && m_client->Connected();
}


int VineyardShmClient::put_data(ObjectIdentifier *output_object_id,
                                const DataBlock *data_block,
                                const KeyValueStore *metadata)
{
    //! Cast the DataBlock and KeyValueStore into internal classes
    auto _data_block = dynamic_cast<const VineyardDataBlock *>(data_block);
    auto _metadata = dynamic_cast<const VineyardParams *>(metadata);

    //! Check if the casting was successful
    if (!_data_block || !_metadata) {

        return -1; // Return error if casting failed
    }

    //! Proceed with the logic using internal_data_block and internal_metadata
    // INSERT_YOUR_LOGIC_HERE
    return 0;
}

int VineyardShmClient::get_data(DataBlock *output_data_block,
                                KeyValueStore *output_metadata,
                                const ObjectIdentifier &identifier)
{
    try {
        // Implement logic to get data from Vineyard
        // This is a placeholder for actual implementation
        return 0;
    } catch (const std::exception &e) {
        //! Handle any exceptions that might occur during data get operation
        return -1;
    }
}

int VineyardShmClient::delete_object(const ObjectIdentifier &identifier,
                                     const KeyValueStore *metadata)
{
    try {
        // Implement logic to delete object from Vineyard
        // This is a placeholder for actual implementation
        return 0;
    } catch (const std::exception &e) {
        //! Handle any exceptions that might occur during object deletion
        return -1;
    }
}


} // namespace redoxi_works::shared_memory
