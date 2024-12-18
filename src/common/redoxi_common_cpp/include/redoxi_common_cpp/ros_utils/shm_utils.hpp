#pragma once

#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_public_msgs/msg/shared_memory_token.hpp>

namespace redoxi_works::shm_utils
{
struct ShmTokenTraits {
    using TokenMsg_t = redoxi_public_msgs::msg::SharedMemoryToken;

    /**
     * @brief Check if the token is valid
     * @param token The token to check
     * @return True if the token is valid, false otherwise
     */
    static bool is_valid(const TokenMsg_t &token)
    {
        // has service name and region key?
        bool is_connectible = !token.region_key.empty() && !token.service_type.empty();

        // has object id or object key?
        bool has_id = token.object_id != token.INVALID_OBJECT_ID || !token.object_key.empty();

        // has object size?
        bool has_size = token.object_size != token.INVALID_OBJECT_SIZE;

        // if all of them are true, the token is valid
        return is_connectible && has_id && has_size;
    }

    // check if the client can read the token or not
    static bool is_client_and_token_compatible(const shared_memory::SharedMemoryClient *client,
                                               const TokenMsg_t &token)
    {
        // null client is not compatible with any token
        if (!client) {
            return false;
        }

        const auto &shm_config = client->get_shm_config();
        bool is_token_valid = is_valid(token);
        bool is_service_type_compatible = shm_config.service_type == token.service_type;
        return is_token_valid && is_service_type_compatible;
    }

    static shared_memory::ObjectIdentifier get_object_identifier(const TokenMsg_t &token)
    {
        shared_memory::ObjectIdentifier output;
        if (token.object_id != token.INVALID_OBJECT_ID) {
            output.id = token.object_id;
        }
        if (!token.object_key.empty()) {
            output.key = token.object_key;
        }
        return output;
    }

    // readable by default client?
    static bool is_readable_by_default_client(const TokenMsg_t &token)
    {
        auto shm_client = shared_memory::SharedMemoryFactory::get_instance().get_default_client().lock();
        if (!shm_client) {
            RDX_INFO_DEV(nullptr, __func__, "{}", "Failed to get default client");
            return false;
        } else {
            RDX_INFO_DEV(nullptr, __func__, "{}", "Got default client");
        }

        auto is_compatible = is_client_and_token_compatible(shm_client.get(), token);
        if (!is_compatible) {
            RDX_INFO_DEV(nullptr, __func__, "Token is not compatible with default client, token.service_type={}, token.region_key={}, shm_config.service_type={}, shm_config.region_key={}",
                         token.service_type, token.region_key, shm_client->get_shm_config().service_type, shm_client->get_shm_config().region_key);
            return false;
        } else {
            RDX_INFO_DEV(nullptr, __func__, "Token is compatible with default client, token.service_type={}, token.region_key={}, shm_config.service_type={}, shm_config.region_key={}",
                         token.service_type, token.region_key, shm_client->get_shm_config().service_type, shm_client->get_shm_config().region_key);
        }

        // check if the token is readable by the default client
        auto object_id = get_object_identifier(token);
        if (object_id.is_empty()) {
            RDX_INFO_DEV(nullptr, __func__, "{}", "Token is empty");
            return false;
        } else {
            RDX_INFO_DEV(nullptr, __func__, "Token is not empty, object id={}", object_id.to_string());
        }

        auto got_data = shm_client->get_data(nullptr, nullptr, object_id) == 0;
        if (!got_data) {
            RDX_INFO_DEV(nullptr, __func__, "Failed to get data from shared memory, object_id={}",
                         object_id.to_string());
        } else {
            RDX_INFO_DEV(nullptr, __func__, "Got data from shared memory, object_id={}",
                         object_id.to_string());
        }

        return got_data;
    }
};
} // namespace redoxi_works::shm_utils