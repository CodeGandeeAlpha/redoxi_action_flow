#include "psg_common/psg_common.hpp"
#include <vineyard/client/client.h>

std::shared_ptr<vineyard::Client> create_v6d_client(const std::string& socket)
{
    std::string v6d_ipc_socket = socket;
    if(v6d_ipc_socket.empty())
        v6d_ipc_socket = "/var/run/vineyard.sock";

    auto v6d_client = std::make_shared<vineyard::Client>();
    VINEYARD_CHECK_OK(v6d_client->Connect(v6d_ipc_socket));

    return v6d_client;
}