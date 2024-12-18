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
};
} // namespace redoxi_works::shm_utils
