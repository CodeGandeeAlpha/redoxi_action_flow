#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
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

std::string VineyardShmClient::get_service_type() const
{
    static const std::string name = config_values::service_types::Vineyard.data();
    return name;
}

std::string VineyardShmClient::get_region_key() const
{
    return m_region_key;
}

rclcpp::Logger VineyardShmClient::_get_logger() const
{
    static rclcpp::Logger logger = rclcpp::get_logger("VineyardShmClient");
    if (m_node) {
        return m_node->get_logger();
    }
    return logger;
}

int VineyardShmClient::connect(const std::string &region_key,
                               const KeyValueStore *)
{
    try {
        m_client = create_v6d_client(region_key);
        m_region_key = region_key;
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
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "disconnecting vineyard client");
        m_client->Disconnect();
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "vineyard client disconnected");
        return 0;
    } catch (const std::exception &e) {
        //! Handle any exceptions that might occur during the process
        return -1;
    }
}

int VineyardShmClient::put_data(vineyard::ObjectID *object_id, const cv::Mat &mat)
{
    RDX_INFO_DEV(_get_logger(), __func__, "putting cv::Mat to vineyard, rows={}, cols={}, type={}", mat.rows, mat.cols, mat.type());
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
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "OK, data put to vineyard");
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        RDX_INFO_DEV(_get_logger(), __func__, "{}", e.what());
        return -1;
    }
}

int VineyardShmClient::put_data(vineyard::ObjectID *output_object_id,
                                const uint8_t *data, size_t size)
{
    RDX_INFO_DEV(_get_logger(), __func__, "putting raw data to vineyard, size={}", size);
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
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "OK, data put to vineyard");
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        RDX_INFO_DEV(_get_logger(), __func__, "{}", e.what());
        return -1;
    }
}

std::shared_ptr<VineyardShmClient::VineyardTensor_u8>
    VineyardShmClient::get_data(vineyard::ObjectID object_id)
{
    RDX_INFO_DEV(_get_logger(), __func__, "getting data from vineyard, object_id={}", object_id);
    try {
        auto tensor = get_tensor_by_v6d_id<uint8_t>(object_id, m_client.get());
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "OK, data retrieved from vineyard");
        return tensor;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        RDX_INFO_DEV(_get_logger(), __func__, "{}", e.what());
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
    RDX_INFO_DEV(_get_logger(), __func__, "{}", "putting data to vineyard");
    //! Cast the DataBlock and KeyValueStore into internal classes
    auto _data_block = dynamic_cast<const VineyardDataBlock *>(data_block);
    if (data_block != nullptr && _data_block == nullptr) {
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "data block type mismatch, should be VineyardDataBlock");
        return -1;
    }

    auto _metadata = dynamic_cast<const VineyardParams *>(metadata);
    if (metadata != nullptr && _metadata == nullptr) {
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "metadata type mismatch, should be VineyardParams");
        return -1;
    }

    int put_return_code = 0;
    vineyard::ObjectID object_id;

    cv::Mat output_mat;
    uint8_t *data = nullptr;
    size_t size = 0;
    if (_data_block->get_as_cvmat(&output_mat) == 0) {
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "found cv::Mat, put as cv::Mat");
        // found cv::Mat, put as cv::Mat
        put_return_code = put_data(&object_id, output_mat);
    } else if (_data_block->get_as_bytes_ref(&data, &size) == 0) {
        RDX_INFO_DEV(_get_logger(), __func__, "{}", "found raw data, put as raw data");
        // put as raw data
        put_return_code = put_data(&object_id, data, size);
    }

    if (put_return_code == 0 && output_object_id) {
        *output_object_id = {.id = object_id};
    }

    return put_return_code;
}

int VineyardShmClient::get_data(DataBlock *output_data_block,
                                KeyValueStore *output_metadata,
                                const ObjectIdentifier &identifier)
{
    if (!m_client) {
        RDX_RAISE_ERROR("{}", "client is not connected");
    }

    try {
        // data block must be VineyardDataBlock, metadata must be VineyardParams, if they are not nullptr
        VineyardDataBlock *data_block = nullptr;
        VineyardParams *metadata = nullptr;

        if (output_data_block) {
            data_block = dynamic_cast<VineyardDataBlock *>(output_data_block);
            if (!data_block) {
                RDX_INFO_DEV(_get_logger(), __func__, "{}", "data block type mismatch, should be VineyardDataBlock");
                return -1;
            }
        }
        if (output_metadata) {
            metadata = dynamic_cast<VineyardParams *>(output_metadata);
            if (!metadata) {
                RDX_INFO_DEV(_get_logger(), __func__, "{}", "metadata type mismatch, should be VineyardParams");
                return -1;
            }
        }

        //! Get object ID from identifier
        vineyard::ObjectID object_id = 0;
        if (identifier.id.has_value()) {
            object_id = identifier.id.value();
        } else {
            RDX_RAISE_ERROR("{}", "object id is not set in identifier");
            return -1;
        }

        //! Get object from vineyard
        auto tensor = get_tensor_by_v6d_id<uint8_t>(object_id, m_client.get());
        if (!tensor) {
            RDX_INFO_DEV(_get_logger(), __func__, "{}", "object not found in vineyard");
            return -1;
        }

        //! convert to cv::Mat, note that even if the tensor is raw bytes, it will be still converted to cv::Mat
        cv::Mat mat = from_v6d_tensor_to_cvmat(tensor);
        data_block->from_cvmat(mat);
        data_block->tensor = tensor; // hold this, otherwise data may be deleted by vineyard
        data_block->m_is_writable = false;
        data_block->m_has_remote_data = true;
        data_block->m_has_local_data = false;

        return 0;
    } catch (const std::exception &e) {
        //! Handle any exceptions that might occur during data get operation
        return -1;
    }
}

int VineyardShmClient::delete_object(const ObjectIdentifier &identifier,
                                     const KeyValueStore *metadata)
{
    if (!m_client || !m_client->Connected()) {
        return -1;
    }

    //! Check metadata type if provided
    if (metadata) {
        auto vineyard_params = dynamic_cast<const VineyardParams *>(metadata);
        if (!vineyard_params) {
            RDX_INFO_DEV(_get_logger(), __func__, "{}", "metadata type mismatch, should be VineyardParams");
            return -1;
        }
    }

    //! Get object ID from identifier
    vineyard::ObjectID object_id = 0;
    if (identifier.id.has_value()) {
        object_id = identifier.id.value();
    } else {
        RDX_RAISE_ERROR("{}", "object id is not set in identifier");
        return -1;
    }

    //! Delete object from vineyard
    try {
        VINEYARD_CHECK_OK(m_client->DelData(object_id));
        return 0;
    } catch (const std::exception &e) {
        //! Handle any exceptions that might occur during the process
        return -1;
    }
}

void VineyardShmClient::set_parent_node(rclcpp::Node *node)
{
    //! Store the parent node pointer
    m_node = node;
}

rclcpp::Node *VineyardShmClient::get_parent_node()
{
    return m_node;
}

const rclcpp::Node *VineyardShmClient::get_parent_node() const
{
    return m_node;
}

std::shared_ptr<DataBlock> VineyardShmClient::create_datablock() const
{
    return std::make_shared<VineyardDataBlock>();
}

std::shared_ptr<KeyValueStore> VineyardShmClient::create_kvstore() const
{
    return std::make_shared<VineyardParams>();
}

} // namespace redoxi_works::shared_memory

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(redoxi_works::shared_memory::VineyardShmClient,
                       redoxi_works::shared_memory::SharedMemoryClient)

PLUGINLIB_EXPORT_CLASS(redoxi_works::shared_memory::VineyardShmClient::VineyardDataBlock,
                       redoxi_works::shared_memory::DataBlock)

PLUGINLIB_EXPORT_CLASS(redoxi_works::shared_memory::VineyardShmClient::VineyardParams,
                       redoxi_works::shared_memory::KeyValueStore)
