#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <redoxi_shm_v6d/VineyardShmClient.hpp>
#include <redoxi_shm_v6d/v6d_helpers.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>


namespace redoxi_works::shared_memory
{

VineyardShmClient::VineyardShmClient()
{
    m_expiration_cache.set_on_evict_callback(
        [this](const ObjectIdentifier &object_id, const ExpirationCache &cache) {
            (void)cache;
            delete_object(object_id, nullptr);
            return 0;
        });
    m_expiration_cache.start_auto_evict();
}

VineyardShmClient::~VineyardShmClient()
{
    // reset the expiration cache, evict all expired blocks and stop the auto eviction thread
    m_expiration_cache.reset();
}

int VineyardShmClient::connect(const SharedMemoryConfig &config,
                               std::shared_ptr<KeyValueStore> additional_params)
{
    (void)additional_params;
    try {
        if (additional_params) {
            auto _params = std::dynamic_pointer_cast<VineyardParams>(additional_params);
            if (!_params) {
                RDX_RAISE_ERROR("{}", "additional params type mismatch, should be VineyardParams");
                return -1;
            }
            m_additional_params = _params;
        }

        m_client = create_v6d_client(config.region_key);
        m_config = config;
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during connection
        return -1;
    }

    // start auto eviction thread
    m_expiration_cache.start_auto_evict();
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
        RDX_INFO_DEV(nullptr, __func__, "{}", "disconnecting vineyard client");
        m_client->Disconnect();
        RDX_INFO_DEV(nullptr, __func__, "{}", "vineyard client disconnected");
        return 0;
    } catch (const std::exception &e) {
        //! Handle any exceptions that might occur during the process
        return -1;
    }
}

int VineyardShmClient::put_data(vineyard::ObjectID *object_id, const cv::Mat &mat)
{
    RDX_INFO_DEV(nullptr, __func__, "putting cv::Mat to vineyard, rows={}, cols={}, type={}", mat.rows, mat.cols, mat.type());
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
        RDX_INFO_DEV(nullptr, __func__, "{}", "OK, data put to vineyard");
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        RDX_INFO_DEV(nullptr, __func__, "{}", e.what());
        return -1;
    }
}

int VineyardShmClient::put_data(vineyard::ObjectID *output_object_id,
                                const uint8_t *data, size_t size)
{
    RDX_INFO_DEV(nullptr, __func__, "putting raw data to vineyard, size={}", size);
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
        RDX_INFO_DEV(nullptr, __func__, "{}", "OK, data put to vineyard");
        return 0;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        RDX_INFO_DEV(nullptr, __func__, "{}", e.what());
        return -1;
    }
}

std::shared_ptr<VineyardShmClient::VineyardTensor_u8>
    VineyardShmClient::get_data(vineyard::ObjectID object_id)
{
    RDX_INFO_DEV(nullptr, __func__, "getting data from vineyard, object_id={}", object_id);
    try {
        auto tensor = get_tensor_by_v6d_id<uint8_t>(object_id, m_client.get());
        RDX_INFO_DEV(nullptr, __func__, "{}", "OK, data retrieved from vineyard");
        return tensor;
    } catch (const std::exception &e) {
        // Handle any exceptions that might occur during the process
        RDX_INFO_DEV(nullptr, __func__, "{}", e.what());
        return nullptr;
    }
}

bool VineyardShmClient::is_connected() const
{
    return m_client != nullptr && m_client->Connected();
}

const SharedMemoryConfig &VineyardShmClient::get_shm_config() const
{
    return m_config;
}

std::shared_ptr<const KeyValueStore> VineyardShmClient::get_connection_params() const
{
    return m_additional_params;
}

int VineyardShmClient::put_data(ObjectIdentifier *output_object_id,
                                const DataBlock *data_block,
                                const KeyValueStore *metadata,
                                std::optional<ShmPutOptions> put_options)
{
    RDX_INFO_DEV(nullptr, __func__, "{}", "putting data to vineyard");
    //! Cast the DataBlock and KeyValueStore into internal classes
    auto _data_block = dynamic_cast<const VineyardDataBlock *>(data_block);
    if (data_block != nullptr && _data_block == nullptr) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "data block type mismatch, should be VineyardDataBlock");
        return -1;
    }

    auto _metadata = dynamic_cast<const VineyardParams *>(metadata);
    if (metadata != nullptr && _metadata == nullptr) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "metadata type mismatch, should be VineyardParams");
        return -1;
    }

    int put_return_code = 0;
    vineyard::ObjectID object_id;

    cv::Mat output_mat;
    uint8_t *data = nullptr;
    size_t size = 0;
    if (_data_block->get_as_cvmat(&output_mat) == 0) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "found cv::Mat, put as cv::Mat");
        // found cv::Mat, put as cv::Mat
        put_return_code = put_data(&object_id, output_mat);
    } else if (_data_block->get_as_bytes_ref(&data, &size) == 0) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "found raw data, put as raw data");
        // put as raw data
        put_return_code = put_data(&object_id, data, size);
    }

    if (put_return_code == 0 && output_object_id) {
        *output_object_id = {.id = object_id};
    }

    // register expiration config
    if (put_return_code == 0) {
        ShmPutOptions opt;
        if (put_options.has_value()) {
            // use the provided put_options if possible
            opt = put_options.value();
        }

        if (!opt.alive_duration.has_value()) {
            // no value? use the default alive duration from m_config
            opt.alive_duration = m_config.default_alive_duration;
        }

        if (!opt.alive_duration.has_value()) {
            // still no value? use the max alive duration from m_config
            opt.alive_duration = m_config.max_alive_duration;
        }

        // expiration is only activated if alive duration is set
        if (opt.alive_duration.has_value()) {
            // clamp the alive duration to the max alive duration from m_config
            if (m_config.max_alive_duration.has_value()) {
                opt.alive_duration = std::min(opt.alive_duration.value(), m_config.max_alive_duration.value());
            }

            // add to expiration cache
            m_expiration_cache.add_memory_block({.id = object_id}, opt);
        }
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
                RDX_INFO_DEV(nullptr, __func__, "{}", "data block type mismatch, should be VineyardDataBlock");
                return -1;
            }
        }
        if (output_metadata) {
            metadata = dynamic_cast<VineyardParams *>(output_metadata);
            if (!metadata) {
                RDX_INFO_DEV(nullptr, __func__, "{}", "metadata type mismatch, should be VineyardParams");
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
            RDX_INFO_DEV(nullptr, __func__, "{}", "object not found in vineyard");
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
    // remove it from cache
    m_expiration_cache.remove_memory_block(identifier, false);
    return _delete_object(identifier, metadata);
}

int VineyardShmClient::_delete_object(const ObjectIdentifier &identifier,
                                      const KeyValueStore *metadata)
{
    if (!m_client || !m_client->Connected()) {
        return -1;
    }

    //! Check metadata type if provided
    if (metadata) {
        auto vineyard_params = dynamic_cast<const VineyardParams *>(metadata);
        if (!vineyard_params) {
            RDX_INFO_DEV(nullptr, __func__, "{}", "metadata type mismatch, should be VineyardParams");
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
std::shared_ptr<DataBlock> VineyardShmClient::create_datablock() const
{
    return std::make_shared<VineyardDataBlock>();
}

std::shared_ptr<KeyValueStore> VineyardShmClient::create_kvstore() const
{
    return std::make_shared<VineyardParams>();
}

void VineyardShmClient::_evict_expired_data_blocks(bool force)
{
    m_expiration_cache.evict_expired_memory_blocks(force);
}

void VineyardShmClient::_set_auto_evict_enabled(bool enable)
{
    if (enable) {
        m_expiration_cache.start_auto_evict();
    } else {
        m_expiration_cache.stop_auto_evict();
    }
}

bool VineyardShmClient::_get_auto_evict_enabled() const
{
    return m_expiration_cache.is_auto_evict_enabled();
}

void VineyardShmClient::set_default_alive_duration(std::optional<TimeUnit_t> default_alive_duration)
{
    m_config.default_alive_duration = default_alive_duration;
}

void VineyardShmClient::set_max_alive_duration(std::optional<TimeUnit_t> max_alive_duration)
{
    m_config.max_alive_duration = max_alive_duration;
}

std::optional<VineyardShmClient::TimeUnit_t> VineyardShmClient::get_default_alive_duration() const
{
    return m_config.default_alive_duration;
}

std::optional<VineyardShmClient::TimeUnit_t> VineyardShmClient::get_max_alive_duration() const
{
    return m_config.max_alive_duration;
}

} // namespace redoxi_works::shared_memory

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(redoxi_works::shared_memory::VineyardShmClient,
                       redoxi_works::shared_memory::SharedMemoryClient)

PLUGINLIB_EXPORT_CLASS(redoxi_works::shared_memory::VineyardShmClient::VineyardDataBlock,
                       redoxi_works::shared_memory::DataBlock)

PLUGINLIB_EXPORT_CLASS(redoxi_works::shared_memory::VineyardShmClient::VineyardParams,
                       redoxi_works::shared_memory::KeyValueStore)
