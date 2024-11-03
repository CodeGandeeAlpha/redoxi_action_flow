#include <redoxi_video_reader/base/VideoReaderBaseTypes_v2.hpp>
#include <redoxi_video_reader/base/VideoReaderBase_v2.hpp>
#include <iostream>
#include <json_struct/json_struct.h>

namespace rdx = redoxi_works;
namespace v2 = rdx::video_reader_base_v2;
using NodeType = rdx::RedoxiVideoReaderBase_v2;

void export_init_json()
{
    NodeType::InitConfig_t init_config;
    auto &port_spec = init_config.primary_output_spec;
    {
        //! Create some downstream specifications
        NodeType::Downstream_t::DownstreamSpec_t ds1;
        ds1.set_action_name("/video_sink/in/action");
        ds1.set_name("video_sink");

        //! Set the delivery policy
        NodeType::DeliveryRequest_t::DeliveryPolicy_t delivery_policy;
        delivery_policy.set_precondition(rdx::DeliveryPrecondition::DontCare);
        delivery_policy.get_retry_policy().set_number_of_retry(5);
        delivery_policy.get_retry_policy().set_wait_time_between_retry(std::chrono::milliseconds(10));
        delivery_policy.get_retry_policy().set_wait_time_retry_response(std::chrono::milliseconds(5));
        ds1.set_delivery_policy(delivery_policy);

        port_spec.set_downstream_specs({ds1});
    }

    auto json_str = JS::serializeStruct(init_config);
    std::cout << json_str << std::endl;
}

int main()
{
    NodeType::RuntimeConfig_t runtime_config;
    auto json_str = JS::serializeStruct(runtime_config);
    std::cout << json_str << std::endl;
    return 0;
}
