#include <redoxi_common_nodes/base_nodes/v2/OpenCloseNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes::v2
{
OpenCloseNode::OpenCloseNode(const std::string &name, const rclcpp::NodeOptions &options)
    : BaseRosNode(name, options)
{
}

OpenCloseNode::RosLifecycleCallbackReturn_t OpenCloseNode::on_configure(const RosLifecycleState_t &)
{
    // init() if necessary, and then _open()
    if (!check_is_already_init()) {
        int ret = init();
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to init node", __func__);
        }
    }

    auto ret = _open();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to open node", __func__);
    }
    return RosLifecycleCallbackReturn_t::SUCCESS;
}

OpenCloseNode::RosLifecycleCallbackReturn_t OpenCloseNode::on_activate(const RosLifecycleState_t &)
{
    auto ret = _start();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to start node", __func__);
    }

    //! Start step thread
    _start_step_thread();

    return RosLifecycleCallbackReturn_t::SUCCESS;
}

OpenCloseNode::RosLifecycleCallbackReturn_t OpenCloseNode::on_deactivate(const RosLifecycleState_t &)
{
    //! Stop step thread
    _stop_step_thread();

    // Call implementation
    auto ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to stop node", __func__);
    }

    return RosLifecycleCallbackReturn_t::SUCCESS;
}

OpenCloseNode::RosLifecycleCallbackReturn_t OpenCloseNode::on_cleanup(const RosLifecycleState_t &)
{
    auto ret = _close();
    if (ret != 0) {
        RDX_RAISE_ERROR("[f={}] Failed to close node", __func__);
    }
    return RosLifecycleCallbackReturn_t::SUCCESS;
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
    // must be in unconfigured state
    if (get_current_state().id() != RosPrimaryState_t::PRIMARY_STATE_UNCONFIGURED) {
        RDX_RAISE_ERROR("[f={}] Cannot open node when not in unconfigured state", __func__);
    }

    LifecycleNode::configure();
    return _on_opened();
}

int OpenCloseNode::close()
{
    // must be in inactive state
    if (get_current_state().id() != RosPrimaryState_t::PRIMARY_STATE_INACTIVE) {
        RDX_RAISE_ERROR("[f={}] Cannot close node when not in inactive state", __func__);
    }

    LifecycleNode::cleanup();
    return _on_closed();
}

int OpenCloseNode::start()
{
    // must be in inactive state
    if (get_current_state().id() != RosPrimaryState_t::PRIMARY_STATE_INACTIVE) {
        RDX_RAISE_ERROR("[f={}] Cannot start node when not in inactive state", __func__);
    }

    LifecycleNode::activate();
    return _on_started();
}

int OpenCloseNode::stop()
{
    // must be in active state
    if (get_current_state().id() != RosPrimaryState_t::PRIMARY_STATE_ACTIVE) {
        RDX_RAISE_ERROR("[f={}] Cannot stop node when not in active state", __func__);
    }

    LifecycleNode::deactivate();
    return _on_stopped();
}
} // namespace redoxi_works::common_nodes::v2
