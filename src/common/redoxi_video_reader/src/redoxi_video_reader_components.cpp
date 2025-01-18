#include <redoxi_video_reader/VideoSourceFromUrl.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace redoxi_works::node_pack::video_readers
{
using VideoSourceFromUrl = redoxi_works::video_readers::VideoSourceFromUrl;
}

RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::video_readers::VideoSourceFromUrl);