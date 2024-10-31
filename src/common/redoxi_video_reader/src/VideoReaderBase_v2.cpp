// #include <redoxi_video_reader/base/VideoReaderBase_v2.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes_v2.hpp>
#include <json_struct/json_struct.h>
#include <spdlog/spdlog.h>

namespace v2 = redoxi_works::RedoxiVideoReaderBaseTypes_v2;

namespace redoxi_works
{
void test()
{
    v2::MainSpec spec;
    v2::InitConfig config;
    v2::RuntimeConfig runtime_config;
    v2::MainSpec::DeliveryPolicy_t delivery_policy;

    auto out = JS::serializeStruct(delivery_policy);
    spdlog::info("downstream_spec: {}", out);
}
} // namespace redoxi_works
