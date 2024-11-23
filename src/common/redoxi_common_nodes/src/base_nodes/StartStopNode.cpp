#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes
{

StartStopNode::StartStopNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : BaseRosNode(node_name, options)
{
}

std::shared_future<int> StartStopNode::_async_stop()
{
    auto promise = std::make_shared<std::promise<int>>();
    auto future = promise->get_future();

    // stop the step thread
    auto step_thread_future = _async_stop_step_thread();

    // run the task in another thread
    m_async_task_group.run([promise, this, step_thread_future]() {
        // wait for the step thread to stop
        step_thread_future.wait();

        // do normal stop and set the promise
        promise->set_value(stop());
    });

    return future;
}

int StartStopNode::start()
{
    //! If already in STARTED state, just return
    if (m_status == NodeStatusCode::STARTED) {
        RDX_INFO_DEV(this, __func__, true, "{}", "Node already in STARTED state, skipping");
        return 0;
    }
    //! Check state is STOPPED
    if (m_status != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[f={}] Node cannot be started in state {}", __func__, m_status);
    }

    //! Call subclass implementation
    auto ret = _start();
    if (ret != 0) {
        //! Stop step thread if start failed
        RDX_RAISE_ERROR("[f={}] Failed to start node", __func__);
    }

    //! Start step thread
    _start_step_thread();

    //! Update state
    set_status(NodeStatusCode::STARTED);

    return 0;
}

int StartStopNode::stop()
{
    //! If already in STOPPED state, just return
    if (m_status == NodeStatusCode::STOPPED) {
        RDX_INFO_DEV(this, __func__, true, "{}", "Node already in STOPPED state, skipping");
        return 0;
    }
    //! Check state is STARTED
    if (m_status != NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[f={}] Node cannot be stopped in state {}", __func__, m_status);
    }

    //! Stop step thread
    _stop_step_thread();

    //! Call subclass implementation
    auto ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to stop node", __func__);
    }

    //! Update state
    set_status(NodeStatusCode::STOPPED);

    return 0;
}

} // namespace redoxi_works::common_nodes
