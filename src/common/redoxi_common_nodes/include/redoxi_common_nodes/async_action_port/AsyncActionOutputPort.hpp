#pragma once

#include <string>
#include "redoxi_common_nodes/redoxi_common_nodes.hpp"
#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/ImageOutputPortSpec.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <atomic>

namespace redoxi_works
{

using TSpec = async_action_image_output_port::ImageOutputPortSpec;

//! Sends action requests to downstream nodes, asynchronously
//! Thread safe, can be used in multi thread executor
// template <output_port_types::AsyncActionOutputPortSpecConcept TSpec>
class AsyncActionOutputPort : public IStartStopProtocol
{
  private:
    inline static constexpr auto PRINT_THREAD_ID = false;

  public:
    AsyncActionOutputPort();
    ~AsyncActionOutputPort() noexcept = default;

    using MasterSpec_t = TSpec; // master specification of this port
    using TimeUnit_t = typename TSpec::TimeUnit_t;
    using InitConfig_t = typename TSpec::InitConfig_t;
    using SourceData_t = typename TSpec::DeliverySourceData_t;
    using TargetData_t = typename TSpec::DeliveryTargetData_t;
    using DeliveryRequest_t = typename TSpec::DeliveryRequest_t;
    using DeliveryTask_t = typename TSpec::DeliveryTask_t;
    using Downstream_t = typename TSpec::Downstream_t;
    using ActionType_t = typename TSpec::ActionType_t;

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
    int init(std::shared_ptr<InitConfig_t> init_config)
    {
        // check init_config, should not be set yet
        RDX_ASSERT_CHECK_TRUE(m_init_config == nullptr, "[{}] init_config is already set", __func__);

        // state must be BEFORE_INIT
        RDX_ASSERT_CHECK_TRUE(m_status == NodeStatusCode::BEFORE_INIT, "[{}] state must be BEFORE_INIT", __func__);

        // assign init config
        m_init_config = init_config;

        // do internal initialization
        {
            auto ret = _create_task_delivery_graph();
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to create task delivery graph", __func__);
            }
        }

        // connect to downstreams
        {
            auto ret = _connect_to_downstreams();
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to connect to downstreams", __func__);
            }
        }

        // state transition: BEFORE_INIT -> STOPPED
        _set_status_code(NodeStatusCode::STOPPED);

        return 0;
    }

    /**
     * @brief Start the port, begin sending action data to downstream nodes
     * @note you cannot modify the downstream specs after starting
     * @note state transition: STOPPED -> STARTED
     * @return 0 if success, otherwise return error code
     */
    int start() override
    {
        // already started? skip
        if (m_status == NodeStatusCode::STARTED) {
            return 0;
        }

        // custom start logic
        {
            auto ret = _start();
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to start the port", __func__);
            }
        }

        // state transition: STOPPED -> STARTED
        _set_status_code(NodeStatusCode::STARTED);

        return 0;
    }

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
    std::shared_ptr<const InitConfig_t> get_init_config() const
    {
        return m_init_config;
    }
    std::shared_ptr<InitConfig_t> get_init_config()
    {
        return m_init_config;
    }

    //! Get the status code of the port
    //! @return The status code
    int get_status() const
    {
        return m_status;
    }

  protected:
    //! for subclass to implement, executed during start(), before state transition
    //! @return 0 if success, otherwise return error code, which will cause start() to fail
    virtual int _start()
    {
        return 0;
    }

  protected:
    //! Create the task delivery graph
    //! @return 0 if success, otherwise return error code
    virtual int _create_task_delivery_graph()
    {
        constexpr auto PRINT_THREAD_ID = false;

        // create graph and node
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Creating graph ...");
        m_delivery_graph = std::make_shared<tbb::flow::graph>();
        auto &g = *m_delivery_graph;

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Creating frame delivery node ...");
        m_delivery_task_node = std::make_shared<
            async_processor::SingleBufferExecNode<DeliveryTask_t>>(g);
        auto &node = *m_delivery_task_node;
        {
            auto is_built = node.is_built();
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Frame delivery node is built: {}", is_built ? "true" : "false");
        }

        // set node params
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting node params ...");
        auto buffer_size = m_init_config->get_num_buffer_requests();
        node.set_input_data_buffer_size(buffer_size);
        node.set_preserve_order(m_init_config->get_preserve_request_order());

        // sync mode, all functions are executed in the graph
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting node to sync mode ...");
        node.set_use_async_callback(false);
        using DeliveryTaskNode_t = async_processor::SingleBufferExecNode<DeliveryTask_t>;
        using WorkInput_t = DeliveryTaskNode_t::InputWithTokens_t;
        using WorkOutput_t = DeliveryTaskNode_t::OutputWithTokens_t;

        // setup work function, nothing to do because during work function
        // frames are out of order
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting work function ...");
        node.set_work_function(
            [this](const WorkInput_t &input, WorkOutput_t &output) -> int {
                // copy input to output
                auto &out_payload = std::get<0>(output);
                const auto &in_payload = std::get<0>(input);
                auto ret = this->_do_frame_delivery_preprocess(in_payload, out_payload);
                if (ret != 0) {
                    RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Failed to preprocess frame, error code: {}", ret);
                }

                return ret;
            });

        // output callback
        // send frame to downstreams
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting output callback ...");
        node.set_output_callback(
            [this](const WorkOutput_t &output) -> int {
                auto &out_payload = std::get<0>(output);
                auto ret = this->_do_frame_delivery_main(out_payload);
                if (ret != 0) {
                    RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Failed to deliver frame, error code: {}", ret);
                }
                return ret;
            });

        // build the node
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Building frame delivery node ...");
        node.build();

        return 0;
    }

    //! Connect to downstream nodes
    //! @return 0 if success, otherwise return error code
    virtual int _connect_to_downstreams()
    {
        //! Ensure m_init_config is set before connecting to downstreams
        RDX_ASSERT_CHECK_TRUE(m_init_config != nullptr,
                              "[{}] Init config is not set", __func__);

        // find and connect to downstreams
        m_downstreams.clear();
        for (auto &it : m_init_config->get_downstream_specs()) {
            auto ds = std::make_shared<Downstream_t>();
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Initializing downstream ...");
            ds->init_by_spec(it, m_parent_node);
            m_downstreams.push_back(ds);
        }

        return 0;
    }

    //! Set the status code of the port
    void _set_status_code(int status_code)
    {
        m_status = status_code;
    }

    virtual void _create_frame_delivery_task(std::shared_ptr<DeliveryRequest_t> request, DeliveryTask_t &task_output)
    {
        task_output.set_request(request);
    }


    /**
     * @brief Publish debug message when failed to send to downstream
     * @param task The delivery task containing request information
     * @param ds The downstream node to publish to
     * @param ith_attempt Current attempt number
     * @param max_attempts Maximum number of attempts allowed
     * @return 0 on success, otherwise return error code
     */
    virtual int _debug_publish_failed_to_send_to_downstream(const DeliveryTask_t &task,
                                                            std::shared_ptr<Downstream_t> ds,
                                                            int64_t ith_attempt,
                                                            int64_t max_attempts)
    {
        auto req = task.get_request();
        SourceData_t::PublishMessageType_t pub_msg;
        req->get_source_data()->to_publish_message(pub_msg);
        auto pub = ds->get_debug_publisher_failed();
        if (pub != nullptr) {
            auto s = fmt::format("[FAILED] attempt {}/{}", ith_attempt, max_attempts);
            pub->publish(pub_msg, s);
        }
        return 0;
    }

    /**
     * @brief Publish debug message when sending to downstream
     * @param task The delivery task containing request information
     * @param ds The downstream node to publish to
     * @param ith_attempt Current attempt number
     * @param max_attempts Maximum number of attempts allowed
     * @return 0 on success, otherwise return error code
     */
    virtual int _debug_publish_sending_to_downstream(const DeliveryTask_t &task,
                                                     std::shared_ptr<Downstream_t> ds,
                                                     int64_t ith_attempt,
                                                     int64_t max_attempts)
    {
        auto req = task.get_request();
        SourceData_t::PublishMessageType_t pub_msg;
        req->get_source_data()->to_publish_message(pub_msg);
        auto pub = ds->get_debug_publisher_sending();
        if (pub != nullptr) {
            auto s = fmt::format("[SENDING] attempt {}/{}", ith_attempt, max_attempts);
            pub->publish(pub_msg, s);
        }
        return 0;
    }

    /**
     * @brief Publish debug message when sent to downstream
     * @param task The delivery task containing request information
     * @param ds The downstream node to publish to
     * @param ith_attempt Current attempt number
     * @param max_attempts Maximum number of attempts allowed
     * @return 0 on success, otherwise return error code
     */
    virtual int _debug_publish_sent_to_downstream(const DeliveryTask_t &task,
                                                  std::shared_ptr<Downstream_t> ds,
                                                  int64_t ith_attempt,
                                                  int64_t max_attempts)
    {
        auto req = task.get_request();
        SourceData_t::PublishMessageType_t pub_msg;
        req->get_source_data()->to_publish_message(pub_msg);
        auto pub = ds->get_debug_publisher_succeeded();
        if (pub != nullptr) {
            auto s = fmt::format("[SUCCEEDED] attempt {}/{}", ith_attempt, max_attempts);
            pub->publish(pub_msg, s);
        }
        return 0;
    }

    //! preprocess the delivery task, and transform it to the target data
    virtual int _do_frame_delivery_preprocess(
        const DeliveryTask_t &task_input,
        DeliveryTask_t &task_output)
    {
        return 0;
    }

    //! main delivery logic
    virtual int _do_frame_delivery_main(const DeliveryTask_t &task)
    {
        return 0;
    }

    //! delivery data to downstream, with retry logic
    virtual int _deliver_data_with_retry(const TargetData_t &target_data,
                                         std::shared_ptr<Downstream_t> ds)
    {
        return 0;
    }

    //! send a single piece of data to downstream
    virtual int _send_data_to_downstream(const TargetData_t &target_data,
                                         std::shared_ptr<Downstream_t> ds,
                                         TimeUnit_t timeout)
    {
        return 0;
    }

    //! Ping the downstream node
    //! @return true if success, otherwise false
    virtual bool _ping(std::shared_ptr<Downstream_t> ds, TimeUnit_t timeout)
    {
        return false;
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

    // the parent node
    rclcpp::Node *m_parent_node = nullptr;

  private:
    //! used tbb to process the delivery tasks
    std::shared_ptr<DeliveryTaskNode_t> m_delivery_task_node;
    std::shared_ptr<tbb::flow::graph> m_delivery_graph;
    tbb::task_group m_task_group; // all async tasks
};


} // namespace redoxi_works