#pragma once

#include <string>
#include <optional>
#include "redoxi_common_nodes/redoxi_common_nodes.hpp"
#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_nodes/AsyncActionPortTypes.hpp>
#include <atomic>
#include <functional>

namespace redoxi_works
{

//! Sends action requests to downstream nodes, asynchronously
//! Thread safe, can be used in multi thread executor
class AsyncActionOutputPort : public IStartStopProtocol
{
  public:
    AsyncActionOutputPort();
    ~AsyncActionOutputPort() noexcept = default;

    using InitConfig_t = AsyncActionPortTypes::InitConfig;
    using SourceData_t = AsyncActionPortTypes::DeliverySourceData;
    using TargetData_t = AsyncActionPortTypes::DeliveryTargetData;
    using DeliveryRequest_t = AsyncActionPortTypes::DeliveryRequest;
    using DeliveryTask_t = AsyncActionPortTypes::DeliveryTask;
    using Downstream_t = AsyncActionPortTypes::Downstream;

  protected:
    using DeliveryTaskNode_t = async_processor::SingleBufferExecNode<DeliveryTask_t>;

  public:
    //! Try to push a request to the port
    //! @return true if success, otherwise false
    bool try_push_request(std::shared_ptr<DeliveryRequest_t> request);

  public:
    /**
     * @brief Initialize the port
     * @note state transition: BEFORE_INIT -> STOPPED
     * @return 0 if success, otherwise return error code
     */
    void init(std::shared_ptr<InitConfig_t> init_config);

    /**
     * @brief Start the port, begin sending action data to downstream nodes
     * @note you cannot modify the downstream specs after starting
     * @note state transition: STOPPED -> STARTED
     * @return 0 if success, otherwise return error code
     */
    int start() override;

    /**
     * @brief Stop the port from action, you can then modify the downstream specs
     * @note state transition: STARTED -> STOPPED
     * @return 0 if success, otherwise return error code
     */
    int stop() override;

    // publish to debug topics?
    void set_publish_to_debug_topic(bool enable)
    {
        m_publish_to_debug_topic = enable;
    }

    bool get_publish_to_debug_topic() const
    {
        return m_publish_to_debug_topic;
    }

    // get the init config
    std::shared_ptr<const InitConfig_t> get_init_config() const;
    std::shared_ptr<InitConfig_t> get_init_config();

  protected:
    //! Initialize the port
    void _init_impl();

    //! Set the status code of the port
    void _set_status_code(int status_code)
    {
        m_status = status_code;
    }

  protected:
    // the status of the port, can be one of the following:
    // BEFORE_INIT, STARTED, STOPPED
    std::atomic<int> m_status = NodeStatusCode::BEFORE_INIT;

    // publish to debug topics?
    std::atomic<bool> m_publish_to_debug_topic = false;

    // init config
    std::shared_ptr<InitConfig_t> m_init_config;

    // downstreams
    std::vector<std::shared_ptr<Downstream_t>> m_downstreams;

  private:
    //! used tbb to process the delivery tasks
    std::shared_ptr<DeliveryTaskNode_t> m_delivery_task_node;
    std::shared_ptr<tbb::flow::graph> m_delivery_graph;
    tbb::task_group m_task_group; // all async tasks
};

} // namespace redoxi_works