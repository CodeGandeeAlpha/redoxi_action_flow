// single input data with buffer

#pragma once
#include <redoxi_common_cpp/async_processor/common.hpp>
#include <functional>
#include <tbb/tbb.h>
#include <cassert>
#include <type_traits>

namespace redoxi_works
{

namespace async_processor
{

// using InputDataType = DummyInputData;
// using OutputDataType = DummyOutputData;
// using InputDataTokenType = DefaultInputDataToken;
// using ExecuteTokenType = DefaultExecToken;

//! @class SingleBufferExecNode
//! @brief A node that processes input data in a single buffer, supporting both synchronous and asynchronous execution.
//!
//! This class provides a flexible way to process input data, with options for synchronous or asynchronous execution,
//! preserving order, and controlling concurrency. It is designed to be used with TBB's flow graph.
//!
//! @tparam InputDataType The type of input data to be processed.
//! @tparam OutputDataType The type of output data produced after processing.
//! @tparam InputDataTokenType The type of token associated with input data (default: DefaultInputDataToken).
//! @tparam ExecuteTokenType The type of execution token (default: DefaultExecToken).
//!
//! @example
//! Here's a simplified example of how to use SingleBufferExecNode in both synchronous and asynchronous modes:
//!
//! ```cpp
//! tbb::flow::graph g;
//! using Node_t = redoxi_works::async_processor::SingleBufferExecNode<InputType, OutputType>;
//!
//! void run_test(bool is_async)
//! {
//!     Node_t node(g);
//!     node.set_is_async(is_async);
//!     node.set_execute_token_size(10);
//!     node.set_input_data_buffer_size(50);
//!
//!     auto work_func = [](const Node_t::InputWithTokens_t &input,
//!                         Node_t::OutputWithTokens_t &output) -> int {
//!         // Process input and produce output
//!         std::get<0>(output).result = std::get<0>(input).value * 2;
//!         return 0; // Success
//!     };
//!
//!     if (is_async) {
//!         node.set_work_function_async([&](const Node_t::InputWithTokens_t &input,
//!                                          Node_t::OutputWithTokens_t &output,
//!                                          typename Node_t::AsyncWorkNode_t::gateway_type &gateway) {
//!             work_func(input, output);
//!             gateway.try_put(output);
//!         });
//!     } else {
//!         node.set_work_function(work_func);
//!     }
//!
//!     auto output_callback = [](const Node_t::OutputWithTokens_t &output) {
//!         // Handle the output
//!         std::cout << "Result: " << std::get<0>(output).result << std::endl;
//!         return 0; // Success
//!     };
//!
//!     if (is_async) {
//!         node.set_output_callback_async([&](const Node_t::OutputWithTokens_t &output,
//!                                            typename Node_t::AsyncOutputCallbackNode_t::gateway_type &gateway) {
//!             output_callback(output);
//!             gateway.try_put(output);
//!         });
//!     } else {
//!         node.set_output_callback(output_callback);
//!     }
//!
//!     node.build();
//!
//!     // Use the node
//!     for (int i = 0; i < 10; ++i) {
//!         InputType input_data{i};
//!         node.put_data(input_data);
//!     }
//!
//!     g.wait_for_all();
//! }
//!
//! int main()
//! {
//!     run_test(false); // Synchronous mode
//!     run_test(true);  // Asynchronous mode
//!     return 0;
//! }
//! ```
//!
//! This example demonstrates how to create, configure, and use a SingleBufferExecNode
//! for both synchronous and asynchronous processing of input data.
template <typename InputDataType,
          typename OutputDataType = DummyOutputData,
          typename InputDataTokenType = DefaultInputDataToken,
          typename ExecuteTokenType = DefaultExecToken,
          typename = std::enable_if_t<std::is_base_of_v<DefaultInputDataToken, InputDataTokenType>>,
          typename = std::enable_if_t<std::is_base_of_v<DefaultExecToken, ExecuteTokenType>>>
class SingleBufferExecNode : public tbb::flow::composite_node<
                                 std::tuple<InputDataType>,
                                 std::tuple<std::tuple<OutputDataType, InputDataTokenType>>>
{
  public:
    using InputData_t = InputDataType;
    using OutputData_t = OutputDataType;

    using InputWithExecToken_t = std::tuple<InputDataType, ExecuteTokenType>;
    using InputWithTokens_t = std::tuple<InputDataType, InputDataTokenType, ExecuteTokenType>;
    using OutputWithTokens_t = std::tuple<OutputDataType, InputDataTokenType, ExecuteTokenType>;
    using OutputWithDataToken_t = std::tuple<OutputDataType, InputDataTokenType>;

    //! use when is_async is true
    using AsyncWorkNode_t = tbb::flow::async_node<InputWithTokens_t, OutputWithTokens_t>;
    using AsyncOutputCallbackNode_t = tbb::flow::async_node<OutputWithTokens_t, OutputWithTokens_t>;

    /**
     * @brief Work function type, update output by input,
     *        (input, output) -> error_code, error_code = 0 means success
     * @note The work function is executed in parallel with other work nodes,
     *       so you HAVE TO care about the thread safety.
     */
    using WorkFunction_t = std::function<int(const InputWithTokens_t &, OutputWithTokens_t &)>;
    using WorkFunctionAsync_t = std::function<
        void(const InputWithTokens_t &, OutputWithTokens_t &, typename AsyncWorkNode_t::gateway_type &)>;

    /**
     * @brief User callback over the output data,
     *        (output_data) -> error_code, error_code = 0 means success
     * @note The output callback is executed in parallel with other output callback nodes,
     *       so you HAVE TO care about the thread safety.
     */
    using OutputCallback_t = std::function<int(const OutputWithTokens_t &)>;
    using OutputCallbackAsync_t = std::function<
        void(const OutputWithTokens_t &, typename AsyncOutputCallbackNode_t::gateway_type &)>;

    using CompositeNode_t = tbb::flow::composite_node<
        std::tuple<InputDataType>,
        std::tuple<OutputWithDataToken_t>>;

  public:
    /**
     * @brief Put data into the node
     * @param data The input data to be put into the node
     * @param bypass_limit If true, bypasses the input buffer limit, directly put data into the internal buffer.
     * This will very likely succeed, but may cause memory issue if not managed properly.
     * In order to clean up the buffer, you can use graph.wait_for_all() to wait for all the data to be processed.
     * @return true if the data was successfully put into the node, false otherwise
     *
     * This function will overwrite the previous data if successful.
     */
    virtual bool put_data(const InputDataType &data, bool bypass_limit = false)
    {
        if (m_input_data_node) {
            if (bypass_limit)
                return m_input_buffer_node->try_put(data);
            else
                return m_input_data_node->try_put(data);
        }

        return false;
    }

  public:
    SingleBufferExecNode(tbb::flow::graph &g,
                         size_t input_data_buffer_size = 1,
                         bool use_async_callback = false,
                         bool preserve_order = true,
                         bool is_serial = false,
                         size_t execute_token_size = DEFAULT_EXECUTE_TOKEN_SIZE)
        : CompositeNode_t(g)
    {
        m_is_serial = is_serial;
        m_use_async_callback = use_async_callback;
        m_preserve_order = preserve_order;
        m_execute_token_size = execute_token_size;
        m_next_sequence_number = std::make_shared<std::atomic<std::size_t>>(0);
        m_input_data_buffer_size = input_data_buffer_size;
    }

    virtual ~SingleBufferExecNode() = default;

    //! Build the node, must be called once and only once before the node is used
    virtual void build()
    {
        // check if the node is already built
        assert(m_execute_token_buf == nullptr);

        // set the concurrency type by the node type
        decltype(tbb::flow::serial) concurrency_type = m_is_serial ? tbb::flow::serial : tbb::flow::unlimited;

        // build the graph
        tbb::flow::graph &g = this->my_graph;

        // buffer the execute tokens
        m_execute_token_buf = std::make_shared<ExecTokenBuf_t>(g);

        // overwrite the input data
        m_input_data_node = std::make_shared<InputDataNode_t>(g, m_input_data_buffer_size);

        // buffer the input data
        m_input_buffer_node = std::make_shared<InputBufferNode_t>(g);

        // join the input data
        m_input_join_node = std::make_shared<InputJoinNode_t>(g);

        // reset the input gate token buffer limiter
        m_input_stamp_node = std::make_shared<InputStampNode_t>(
            g, tbb::flow::serial,
            [this](const InputWithExecToken_t &input_data, typename InputStampNode_t::output_ports_type &ports) {
                // stamp the input data with the input gate token
                InputWithTokens_t stamped_data = std::make_tuple(std::get<0>(input_data), InputDataTokenType(), std::get<1>(input_data));
                this->_stamp_data_token(stamped_data);

                // output the stamped data, assuming always success
                std::get<0>(ports).try_put(stamped_data);

                // output the continue message, assuming always success
                std::get<1>(ports).try_put(tbb::flow::continue_msg());
            });

        // sequencer node
        m_output_sequencer_node = nullptr;
        if (m_preserve_order)
            m_output_sequencer_node = std::make_shared<OutputSequencerNode_t>(
                g, [](const OutputWithTokens_t &output_data) {
                    return std::get<1>(output_data).sequence_number;
                });

        // work node
        m_work_node = nullptr;
        m_async_work_node = nullptr;
        if (m_use_async_callback)
            m_async_work_node = std::make_shared<AsyncWorkNode_t>(
                g, concurrency_type, [this](const InputWithTokens_t &input_data, typename AsyncWorkNode_t::gateway_type &gateway) {
                    this->_nodefunc_work_async(input_data, gateway);
                });
        else
            m_work_node = std::make_shared<WorkNode_t>(
                g, concurrency_type, [this](const InputWithTokens_t &input_data) {
                    return this->_nodefunc_work(input_data);
                });

        // if sequencer node is applied, then after sequencer node, the output pipeline should be serial
        // otherwise the output will be out of order
        auto output_pipeline_concurrency_type = tbb::flow::unlimited;
        if (m_preserve_order || m_is_serial)
            output_pipeline_concurrency_type = tbb::flow::serial;

        // output node, call user callback
        m_output_callback_node = nullptr;
        m_async_output_callback_node = nullptr;
        if (m_use_async_callback)
            m_async_output_callback_node = std::make_shared<AsyncOutputCallbackNode_t>(
                g, output_pipeline_concurrency_type,
                [this](const OutputWithTokens_t &output_data, typename AsyncOutputCallbackNode_t::gateway_type &gateway) {
                    this->_nodefunc_output_callback_async(output_data, gateway);
                });
        else
            m_output_callback_node = std::make_shared<OutputCallbackNode_t>(
                g, output_pipeline_concurrency_type,
                [this](const OutputWithTokens_t &output_data) {
                    return this->_nodefunc_output_callback(output_data);
                });

        // finalize output, do some predefined work and write to output port
        m_finalize_node = std::make_shared<FinalizeNode_t>(
            g, output_pipeline_concurrency_type,
            [](const OutputWithTokens_t &output_data,
               typename FinalizeNode_t::output_ports_type &ports) {
                // spdlog::info("[GRAPH] Finalize node, output_data={}, input_gate_token={}", std::get<0>(output_data).result, std::get<1>(output_data).sequence_number);
                const auto &output = std::get<0>(output_data);
                const auto &data_token = std::get<1>(output_data);
                const auto &exec_token = std::get<2>(output_data);

                // output to data port
                std::get<0>(ports).try_put(std::make_tuple(output, data_token));

                // return execution token
                std::get<1>(ports).try_put(exec_token);
            });

        // build the main graph

        // (external data input) -> limiter -> buffer node
        tbb::flow::make_edge(*m_input_data_node, *m_input_buffer_node);

        // (input_data, exec_token) -> join_node
        tbb::flow::make_edge(*m_input_buffer_node, tbb::flow::input_port<0>(*m_input_join_node));
        tbb::flow::make_edge(*m_execute_token_buf, tbb::flow::input_port<1>(*m_input_join_node));

        // (input_data, exec_token) -> stamp_node
        tbb::flow::make_edge(*m_input_join_node, *m_input_stamp_node);

        // stamp_node.port[0] -> work_node or async_work_node
        if (m_use_async_callback)
            tbb::flow::make_edge(tbb::flow::output_port<0>(*m_input_stamp_node), *m_async_work_node);
        else
            tbb::flow::make_edge(tbb::flow::output_port<0>(*m_input_stamp_node), *m_work_node);

        // stamp_node.port[1] -> release the input gate token slot
        tbb::flow::make_edge(tbb::flow::output_port<1>(*m_input_stamp_node), m_input_data_node->decrementer());

        // work_node -> (output_sequencer_node) -> finalize_node
        if (m_preserve_order) {
            // if preserve order is requested, work_node -> sequencer_node -> output_callback_node -> finalize_node
            assert(m_output_sequencer_node != nullptr);
            if (m_use_async_callback) {
                tbb::flow::make_edge(*m_async_work_node, *m_output_sequencer_node);
                tbb::flow::make_edge(*m_output_sequencer_node, *m_async_output_callback_node);
                tbb::flow::make_edge(*m_async_output_callback_node, *m_finalize_node);
            } else {
                tbb::flow::make_edge(*m_work_node, *m_output_sequencer_node);
                tbb::flow::make_edge(*m_output_sequencer_node, *m_output_callback_node);
                tbb::flow::make_edge(*m_output_callback_node, *m_finalize_node);
            }
        } else {
            // if preserve order is not requested, work_node -> output_callback_node -> finalize_node
            if (m_use_async_callback) {
                tbb::flow::make_edge(*m_async_work_node, *m_async_output_callback_node);
                tbb::flow::make_edge(*m_async_output_callback_node, *m_finalize_node);
            } else {
                tbb::flow::make_edge(*m_work_node, *m_output_callback_node);
                tbb::flow::make_edge(*m_output_callback_node, *m_finalize_node);
            }
        }

        // finalize_node.port[1] -> execute token buffer
        // return exec token to the buffer, so that it can be reused
        tbb::flow::make_edge(tbb::flow::output_port<1>(*m_finalize_node), *m_execute_token_buf);

        // input.port[0] -> input_data_node, use to refresh data
        // input.port[1] -> input_gate_token_limiter, signal the input data has been updated, ready to be consumed
        // (output_data, input_gate_token) -> output.port[0]
        using CompositeInputPorts_t = typename CompositeNode_t::input_ports_type;
        using CompositeOutputPorts_t = typename CompositeNode_t::output_ports_type;
        this->set_external_ports(
            CompositeInputPorts_t(*m_input_data_node),
            CompositeOutputPorts_t(tbb::flow::output_port<0>(*m_finalize_node)));

        // prepare the node
        prepare();
    }

    //! Generate input gate token, created by user
    virtual void generate_input_gate_token(InputDataTokenType &token)
    {
        // sequence number is only incremented by the finalize node
        token.sequence_number = *m_next_sequence_number;
    }

    //! Set if the node is async, only callable before build()
    virtual void set_use_async_callback(bool use_async_callback)
    {
        assert(!is_built());
        m_use_async_callback = use_async_callback;
    }

    //! Check if the node is async
    virtual bool is_callback_async() const
    {
        assert(!is_built());
        return m_use_async_callback;
    }

    virtual bool is_built() const
    {
        return m_execute_token_buf != nullptr;
    }

    //! Set if the node is serial, only callable before build()
    virtual void set_is_serial(bool is_serial)
    {
        assert(!is_built());
        m_is_serial = is_serial;
    }

    //! Set if the node is preserving the order, only callable before build()
    virtual void set_preserve_order(bool preserve_order)
    {
        assert(!is_built());
        m_preserve_order = preserve_order;
    }

    //! Set the execute token size, only callable before build()
    virtual void set_execute_token_size(size_t execute_token_size)
    {
        assert(!is_built());
        m_execute_token_size = execute_token_size;
    }

    //! Set the input data buffer size, only callable before build()
    virtual void set_input_data_buffer_size(size_t input_data_buffer_size)
    {
        assert(!is_built());
        m_input_data_buffer_size = input_data_buffer_size;
    }

    //! Get the input data buffer size
    virtual size_t get_input_data_buffer_size() const
    {
        return m_input_data_buffer_size;
    }

    //! Generate input gate token
    virtual InputDataTokenType generate_input_gate_token()
    {
        InputDataTokenType token;
        generate_input_gate_token(token);
        return token;
    }

    //! Check if the node is serial
    virtual bool is_serial() const
    {
        return m_is_serial;
    }

    //! Check if the node is preserving the order
    virtual bool is_preserving_order() const
    {
        return m_preserve_order;
    }

    //! Get the execute token size
    virtual size_t get_execute_token_size() const
    {
        return m_execute_token_size;
    }

    //! Set the work function, must be done before build()
    virtual void set_work_function(const WorkFunction_t &work_function)
    {
        assert(!is_built());
        m_work_function = work_function;
    }

    //! Set the work function for async mode, must be done before build(), only useful when is_async is true
    virtual void set_work_function_async(const WorkFunctionAsync_t &work_function)
    {
        assert(!is_built());
        m_work_function_async = work_function;
    }

    //! Set the output callback, must be done before build()
    virtual void set_output_callback(const OutputCallback_t &output_callback)
    {
        assert(!is_built());
        m_output_callback = output_callback;
    }

    //! Set the output callback for async mode, must be done before build(), only useful when is_async is true
    virtual void set_output_callback_async(const OutputCallbackAsync_t &output_callback)
    {
        assert(!is_built());
        m_output_callback_async = output_callback;
    }

    //! call this after the graph is reset, or otherwise the node will be blocked
    virtual void prepare()
    {
        *m_next_sequence_number = 0;

        // reset the execute token buffer
        if (m_execute_token_buf) {
            m_execute_token_buf->reset();
            for (size_t i = 0; i < m_execute_token_size; i++)
                while (!m_execute_token_buf->try_put(ExecuteTokenType()))
                    ;
        }
    }

  protected:
    //! Stamp data token, mark it as consumed
    virtual void _stamp_data_token(InputWithTokens_t &input_data)
    {
        // assign the sequence number to the token
        std::get<1>(input_data).sequence_number = (*m_next_sequence_number)++;
    }

    //! Function to process work output
    virtual OutputWithTokens_t _nodefunc_work(const InputWithTokens_t &input_data)
    {
        OutputWithTokens_t output_data;

        // copy tokens
        std::get<1>(output_data) = std::get<1>(input_data);
        std::get<2>(output_data) = std::get<2>(input_data);

        // update error code
        auto &exec_token = std::get<2>(output_data);
        if (m_work_function)
            exec_token.error_code = m_work_function(input_data, output_data);

        // done
        return output_data;
    }

    virtual void _nodefunc_work_async(const InputWithTokens_t &input_data,
                                      typename AsyncWorkNode_t::gateway_type &gateway)
    {
        OutputWithTokens_t output_data;

        // copy tokens
        std::get<1>(output_data) = std::get<1>(input_data);
        std::get<2>(output_data) = std::get<2>(input_data);

        if (m_work_function_async) {
            m_work_function_async(input_data, output_data, gateway);
        } else {
            gateway.reserve_wait();
            gateway.try_put(output_data);
            gateway.release_wait();
        }
    }

    //! Function to call user output callback
    virtual OutputWithTokens_t _nodefunc_output_callback(const OutputWithTokens_t &output_data)
    {
        OutputWithTokens_t _output = output_data;
        auto &exec_token = std::get<2>(_output);
        if (m_output_callback)
            exec_token.error_code = m_output_callback(_output);

        return _output;
    }

    virtual void _nodefunc_output_callback_async(const OutputWithTokens_t &output_data,
                                                 typename AsyncOutputCallbackNode_t::gateway_type &gateway)
    {
        if (m_output_callback_async) {
            m_output_callback_async(output_data, gateway);
        } else {
            // nothing to work, just put the output data
            gateway.reserve_wait();
            gateway.try_put(output_data);
            gateway.release_wait();
        }
    }

  protected:
    // Node types

    //! used to provide input data, always overwrite the previous data
    using InputDataNode_t = tbb::flow::limiter_node<InputDataType>;

    //! buffer the input data
    using InputBufferNode_t = tbb::flow::queue_node<InputDataType>;

    //! signals that execution quota is available
    using ExecTokenBuf_t = tbb::flow::queue_node<ExecuteTokenType>;

    //! read input when required tokens are ready
    using InputJoinNode_t = tbb::flow::join_node<InputWithExecToken_t>;

    //! after join, stamp the input data with the input gate token
    using InputStampNode_t = tbb::flow::multifunction_node<
        InputWithExecToken_t,
        std::tuple<InputWithTokens_t, tbb::flow::continue_msg>>;

    //! execute work function
    using WorkNode_t = tbb::flow::function_node<InputWithTokens_t, OutputWithTokens_t>;

    //! output sequencer, only necessary when preserve order is true
    using OutputSequencerNode_t = tbb::flow::sequencer_node<OutputWithTokens_t>;

    //! output callback node which calls the user callback
    using OutputCallbackNode_t = tbb::flow::function_node<OutputWithTokens_t, OutputWithTokens_t>;

    //! finalize output
    using FinalizeNode_t = tbb::flow::multifunction_node<
        OutputWithTokens_t,
        std::tuple<OutputWithDataToken_t, ExecuteTokenType>>;


  protected:
    WorkFunction_t m_work_function;
    WorkFunctionAsync_t m_work_function_async;
    OutputCallback_t m_output_callback;
    OutputCallbackAsync_t m_output_callback_async;

    size_t m_input_data_buffer_size = 1;
    size_t m_execute_token_size = 1;
    bool m_use_async_callback = false;
    //! if true, the output callback will be called in the order of the input data
    //! @note this requires the user to set the sequence number of the input gate token properly
    //! and reset the graph in order to restart the sequencing
    bool m_preserve_order = true;

    //! if true, the work function will be executed in serial
    bool m_is_serial = false;

    //! next sequence number to be used for the output data
    std::shared_ptr<std::atomic<std::size_t>> m_next_sequence_number;

    //! use shared_ptr so that you do not have to create them in constructor
    //! Limiter for execute token buffer
    std::shared_ptr<ExecTokenBuf_t> m_execute_token_buf;

    std::shared_ptr<InputDataNode_t> m_input_data_node;
    std::shared_ptr<InputBufferNode_t> m_input_buffer_node;
    std::shared_ptr<InputJoinNode_t> m_input_join_node;
    std::shared_ptr<InputStampNode_t> m_input_stamp_node;

    std::shared_ptr<WorkNode_t> m_work_node;
    std::shared_ptr<AsyncWorkNode_t> m_async_work_node;
    std::shared_ptr<OutputSequencerNode_t> m_output_sequencer_node;

    std::shared_ptr<OutputCallbackNode_t> m_output_callback_node;
    std::shared_ptr<AsyncOutputCallbackNode_t> m_async_output_callback_node;
    std::shared_ptr<FinalizeNode_t> m_finalize_node;
};

} // namespace async_processor

} // namespace redoxi_works
