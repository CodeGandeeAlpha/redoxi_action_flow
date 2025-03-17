// #define JS_STL_MAP
// #define JS_STL_SET
// #define JS_STL_UNORDERED_MAP
// #define JS_STL_UNORDERED_SET
#include <test_package/_pch.hpp>

#include <json_struct/json_struct.h>
#include <spdlog/spdlog.h>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <optional>
#include <unordered_map>
#include <set>

struct TestStruct {
    int a;
    std::string b;
    std::set<int> c;
    std::unordered_map<std::string, std::string> d;
    std::map<std::string, int> e;
    JS_OBJECT(JS_MEMBER(a), JS_MEMBER(b), JS_MEMBER(c), JS_MEMBER(d), JS_MEMBER(e));
};

int main()
{
    TestStruct test_struct;
    test_struct.a = 1;
    test_struct.b = "hello";
    test_struct.c = {1, 2, 3};
    test_struct.d = {{"a", "b"}, {"c", "d"}};
    test_struct.e = {{"a", 1}, {"b", 2}, {"c", 3}};
    std::string json_str = JS::serializeStruct(test_struct);
    spdlog::info("Serialized struct: {}", json_str);

    TestStruct deserialized_struct;
    JS::ParseContext pc(json_str);
    auto error_code = pc.parseTo(deserialized_struct);
    spdlog::info("Deserialized struct: a = {}, b = {}, c = {}, d = {}, e = {}",
                 deserialized_struct.a, deserialized_struct.b,
                 deserialized_struct.c, deserialized_struct.d,
                 deserialized_struct.e);

    return 0;
}