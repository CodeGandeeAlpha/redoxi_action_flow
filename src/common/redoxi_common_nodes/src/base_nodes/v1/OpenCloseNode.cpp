#include <redoxi_common_nodes/_pch.hpp>

#include <redoxi_common_nodes/base_nodes/v1/OpenCloseNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes::v1
{
OpenCloseNode::OpenCloseNode(const std::string &name, const rclcpp::NodeOptions &options)
    : BaseRosNode(name, options)
{
}

std::shared_future<int> OpenCloseNode::_async_close()
{
    auto promise = std::make_shared<std::promise<int>>();
    auto future = promise->get_future();

    // stop the thread
    auto step_thread_future = _async_stop_step_thread();
    m_async_task_group.run([promise, this, step_thread_future]() {
        // wait for the step thread to stop
        step_thread_future.wait();

        // do normal close and set the promise
        promise->set_value(close());
    });

    return future;
}

std::shared_future<int> OpenCloseNode::_async_stop()
{
    auto promise = std::make_shared<std::promise<int>>();
    auto future = promise->get_future();

    // stop the thread
    auto step_thread_future = _async_stop_step_thread();
    m_async_task_group.run([promise, this, step_thread_future]() {
        // wait for the step thread to stop
        step_thread_future.wait();

        // do normal stop and set the promise
        promise->set_value(stop());
    });

    return future;
}

std::shared_future<int> OpenCloseNode::_async_open()
{
    auto promise = std::make_shared<std::promise<int>>();
    auto future = promise->get_future();

    // run the task in another thread
    m_async_task_group.run([promise, this]() {
        // do normal open and set the promise
        promise->set_value(open());
    });

    return future;
}

std::shared_future<int> OpenCloseNode::_async_start()
{
    auto promise = std::make_shared<std::promise<int>>();
    auto future = promise->get_future();

    // run the task in another thread
    m_async_task_group.run([promise, this]() {
        // do normal start and set the promise
        promise->set_value(start());
    });

    return future;
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

    set_status(NodeStatusCode::STOPPED);
    return _on_opened();
}

int OpenCloseNode::close()
{
    //! If already in CLOSED state, just return
    if (get_status() == NodeStatusCode::CLOSED) {
        RDX_INFO_DEV(this, __func__, true, "{}", "Node already in CLOSED state, skipping");
        return 0;
    }

    //! Check if node is in STOPPED state
    if (get_status() != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[f={}] Cannot close node when not in STOPPED state", __func__);
    }

    //! Call implementation
    int ret = _close();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to close node", __func__);
    }

    set_status(NodeStatusCode::CLOSED);
    return _on_closed();
}

int OpenCloseNode::start()
{
    //! If already in STARTED state, just return
    if (get_status() == NodeStatusCode::STARTED) {
        RDX_INFO_DEV(this, __func__, true, "{}", "Node already in STARTED state, skipping");
        return 0;
    }
    //! Check if node is in STOPPED state
    if (get_status() != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[f={}] Cannot start node when not in STOPPED state", __func__);
    }

    //! Call implementation
    int ret = _start();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to start node", __func__);
    }

    //! Start step thread
    _start_step_thread();

    set_status(NodeStatusCode::STARTED);
    return _on_started();
}

int OpenCloseNode::stop()
{
    //! If already in STOPPED state, just return
    if (get_status() == NodeStatusCode::STOPPED) {
        RDX_INFO_DEV(this, __func__, true, "{}", "Node already in STOPPED state, skipping");
        return 0;
    }
    //! Check if node is in STARTED state
    RDX_INFO_DEV(this, __func__, true, "{}", "Checking node status");
    if (get_status() != NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[f={}] Cannot stop node when not in STARTED state", __func__);
    }

    //! Stop step thread
    RDX_INFO_DEV(this, __func__, true, "{}", "Stopping step thread");
    _stop_step_thread();

    //! Call implementation
    RDX_INFO_DEV(this, __func__, true, "{}", "Calling stop implementation");
    int ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to stop node", __func__);
    }

    RDX_INFO_DEV(this, __func__, true, "{}", "Setting node status to STOPPED");
    set_status(NodeStatusCode::STOPPED);
    return _on_stopped();
}
} // namespace redoxi_works::common_nodes::v1