#include <redoxi_common_cpp/ros_utils/message_conversion.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;

int main()
{
    // tested, they are the same
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    spdlog::info("boost UUID: {}", boost::uuids::to_string(uuid));

    auto redoxi_uuid = rdx::to_ros_uuid_msg(uuid);
    auto boost_uuid = rdx::to_boost_uuid(redoxi_uuid);
    spdlog::info("boost-ros.msg-boost UUID: {}", boost::uuids::to_string(boost_uuid));

    auto redoxi_uuid2 = rdx::to_ros_goal_uuid(boost_uuid);
    auto boost_uuid2 = rdx::to_boost_uuid(redoxi_uuid2);
    spdlog::info("boost-ros.goal-boost UUID: {}", boost::uuids::to_string(boost_uuid2));
    return 0;
}