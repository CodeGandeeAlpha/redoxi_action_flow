#include <redoxi_video_reader/base/VideoReaderBaseTypes_v2.hpp>
#include <redoxi_video_reader/base/VideoReaderBase_v2.hpp>
#include <spdlog/spdlog.h>
#include <iostream>
#include <json_struct/json_struct.h>
#include <map>

namespace rdx = redoxi_works;
namespace v2 = rdx::video_reader_base_v2;
using NodeType = rdx::RedoxiVideoReaderBase_v2;

enum class MyEnum {
    XXX = 1,
    YYY = 2,
    ZZZ = 3,
};

struct MyStruct {
    MyEnum e1 = MyEnum::ZZZ;
    MyEnum e2 = MyEnum::YYY;

    JS_OBJECT(JS_MEMBER(e1), JS_MEMBER(e2));
};

namespace JS
{
template <>
struct TypeHandler<MyEnum> {
    static inline const std::map<std::string, MyEnum> str_to_enum = {
        {"xxx", MyEnum::XXX},
        {"yyy", MyEnum::YYY},
        {"zzz", MyEnum::ZZZ},
    };
    static inline const std::map<MyEnum, std::string> enum_to_str = {
        {MyEnum::XXX, "xxx"},
        {MyEnum::YYY, "yyy"},
        {MyEnum::ZZZ, "zzz"},
    };

    static inline Error to(MyEnum &to_type, ParseContext &context)
    {
        //! First try to parse as string
        std::string str;
        Error err = TypeHandler<std::string>::to(str, context);

        if (err == Error::NoError) {
            //! Convert to lower case for case-insensitive comparison
            std::transform(str.begin(), str.end(), str.begin(), ::tolower);

            auto it = str_to_enum.find(str);
            if (it != str_to_enum.end()) {
                to_type = it->second;
                return Error::NoError;
            }
            return Error::IllegalDataValue;
        }
        return err;
    }

    static inline void from(const MyEnum &from_type, Token &token, Serializer &serializer)
    {
        //! Convert to string using enum_to_str map
        auto it = enum_to_str.find(from_type);
        if (it != enum_to_str.end()) {
            TypeHandler<std::string>::from(it->second, token, serializer);
        }
    }
};
} // namespace JS

void export_init_json();
void export_runtime_json();

int main()
{
    // export_init_json();
    export_runtime_json();
    // MyStruct my_struct;
    // auto json_str = JS::serializeStruct(my_struct);
    // spdlog::info("my_struct: {}", json_str);

    // MyStruct my_struct2;
    // JS::ParseContext context(json_str);
    // JS::Error err = context.parseTo(my_struct2);
    // spdlog::info("my_struct2: {}, {}, err: {}", static_cast<int>(my_struct2.e1), static_cast<int>(my_struct2.e2), static_cast<int>(err));

    // return 0;
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
