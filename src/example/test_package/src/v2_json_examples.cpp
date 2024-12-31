#include <test_package/_pch.hpp>

#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <spdlog/spdlog.h>
#include <iostream>
#include <json_struct/json_struct.h>
#include <map>

namespace rdx = redoxi_works;
using NodeType = rdx::RedoxiVideoReaderBase;

struct MyBase {
    int a;
    JS_OBJECT(JS_MEMBER(a));
};

struct MyDerived : public MyBase {
    int b;
    JS_OBJECT_WITH_SUPER(JS_SUPER(MyBase), JS_MEMBER(b));
};

void export_init_json();
void export_runtime_json();

int main()
{
    MyDerived my_derived;
    my_derived.a = 1;
    my_derived.b = 2;
    auto json_str = JS::serializeStruct(my_derived);
    spdlog::info("json_str: {}", json_str);
    return 0;
}

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

void export_runtime_json()
{
    NodeType::RuntimeConfig_t runtime_config;
    runtime_config.frame_request_policy = NodeType::DeliveryPolicy_t();
    auto json_str = JS::serializeStruct(runtime_config);
    std::cout << json_str << std::endl;
}
