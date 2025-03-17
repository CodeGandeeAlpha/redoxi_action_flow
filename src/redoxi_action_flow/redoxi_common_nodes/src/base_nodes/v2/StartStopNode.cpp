#include <redoxi_common_nodes/_pch.hpp>

#include <redoxi_common_nodes/base_nodes/v2/StartStopNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes::v2
{

// StartStopNode::StartStopNode(const std::string &node_name, const rclcpp::NodeOptions &options)
//     : BaseRosNode(node_name, options)
// {
// }

StartStopNode::RosLifecycleCallbackReturn_t StartStopNode::on_configure(const RosLifecycleState_t &)
{
    if (!check_is_already_init()) {
        auto ret = init();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to init node", __func__);
        }
    }

    RDX_INFO_DEV(this, __func__, false, "{}", "Node configured");
    return RosLifecycleCallbackReturn_t::SUCCESS;
}

StartStopNode::RosLifecycleCallbackReturn_t StartStopNode::on_activate(const RosLifecycleState_t &state)
{
    auto ret = _start();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to start node", __func__);
    }

    //! Start step thread
    _start_step_thread();

    // do parent work
    BaseRosNode::on_activate(state);

    RDX_INFO_DEV(this, __func__, false, "{}", "Node activated");
    return RosLifecycleCallbackReturn_t::SUCCESS;
}

StartStopNode::RosLifecycleCallbackReturn_t StartStopNode::on_deactivate(const RosLifecycleState_t &state)
{
    //! Stop step thread
    _stop_step_thread();

    // Call implementation
    auto ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to stop node", __func__);
    }

    // do parent work
    BaseRosNode::on_deactivate(state);

    RDX_INFO_DEV(this, __func__, false, "{}", "Node deactivated");
    return RosLifecycleCallbackReturn_t::SUCCESS;
}

StartStopNode::RosLifecycleCallbackReturn_t StartStopNode::on_shutdown(const RosLifecycleState_t &state)
{
    // do parent work
    BaseRosNode::on_shutdown(state);
    // perform stop and cleanup
    _stop_step_thread();
    // perform deactivate and cleanup
    auto ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to stop node", __func__);
    }


    RDX_INFO_DEV(this, __func__, false, "{}", "Node shutdown");
    return RosLifecycleCallbackReturn_t::SUCCESS;
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

std::shared_future<int> StartStopNode::_async_start()
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

int StartStopNode::start()
{
    // if in unconfigured state, we need to configure first
    if (get_current_state().id() == RosPrimaryState_t::PRIMARY_STATE_UNCONFIGURED) {
        configure();
    }

    // must be in inactive state
    if (get_current_state().id() != RosPrimaryState_t::PRIMARY_STATE_INACTIVE) {
        RDX_RAISE_ERROR("[f={}] Cannot start node when not in inactive state", __func__);
    }

    // do parent work
    this->activate();

    return _on_started();
}

int StartStopNode::stop()
{
    // must be in active state
    if (get_current_state().id() != RosPrimaryState_t::PRIMARY_STATE_ACTIVE) {
        RDX_RAISE_ERROR("[f={}] Cannot stop node when not in active state", __func__);
    }

    // do parent work
    this->deactivate();

    // clean up the node back to unconfigured state
    // LifecycleNode::cleanup();

    return _on_stopped();
}

} // namespace redoxi_works::common_nodes::v2
