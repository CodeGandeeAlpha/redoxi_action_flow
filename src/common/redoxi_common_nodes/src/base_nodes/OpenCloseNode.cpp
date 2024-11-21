#include <redoxi_common_nodes/base_nodes/OpenCloseNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes
{
OpenCloseNode::OpenCloseNode(const std::string &name, const rclcpp::NodeOptions &options)
    : BaseRosNode(name, options)
{
}

int OpenCloseNode::open()
{
    //! Check if node is in CLOSED state
    if (get_status() != NodeStatusCode::CLOSED) {
        RDX_RAISE_ERROR("[f={}] Cannot open node when not in CLOSED state", __func__);
    }

    //! Call implementation
    int ret = _open();
    if (ret != 0) {
        //! Failed to open node, just raise error and return
        RDX_RAISE_ERROR("[f={}] Failed to open node", __func__);
    }

    set_status(NodeStatusCode::OPENED);
    return 0;
}

int OpenCloseNode::close()
{
    //! Check if node is in OPENED or STOPPED state
    if (get_status() != NodeStatusCode::OPENED && get_status() != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[f={}] Cannot close node when not in OPENED or STOPPED state", __func__);
    }

    //! Call implementation
    int ret = _close();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to close node", __func__);
    }

    set_status(NodeStatusCode::CLOSED);
    return 0;
}

int OpenCloseNode::start()
{
    //! Check if node is in OPENED state
    if (get_status() != NodeStatusCode::OPENED) {
        RDX_RAISE_ERROR("[f={}] Cannot start node when not in OPENED state", __func__);
    }

    //! Call implementation
    int ret = _start();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to start node", __func__);
    }

    //! Start step thread
    _start_step_thread();

    set_status(NodeStatusCode::STARTED);
    return 0;
}

int OpenCloseNode::stop()
{
    //! Check if node is in STARTED state
    if (get_status() != NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[f={}] Cannot stop node when not in STARTED state", __func__);
    }

    //! Stop step thread
    _stop_step_thread();

    //! Call implementation
    int ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to stop node", __func__);
    }

    set_status(NodeStatusCode::STOPPED);
    return 0;
}
} // namespace redoxi_works::common_nodes