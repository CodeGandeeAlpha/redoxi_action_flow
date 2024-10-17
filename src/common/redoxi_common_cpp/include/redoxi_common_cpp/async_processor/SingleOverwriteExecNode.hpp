#pragma once
#include <redoxi_common_cpp/async_processor/common.hpp>
#include <functional>
#include <tbb/tbb.h>
#include <cassert>
#include <type_traits>
#include <spdlog/spdlog.h>

namespace redoxi_works
{

namespace async_processor
{


// useful when development, because clangd fails to auto suggest with template parameters
// using InputDataType = DummyInputData;
// using OutputDataType = DummyOutputData;
// using InputGateTokenType = DefaultInputGateToken;
// using ExecuteTokenType = DefaultExecToken;


// FIXME: it seems that the processor always keep the output order, even if the preserve_order is false

/**
//! Single overwrite execution node, used for single producer and single consumer
//! @tparam InputDataType The type of the input data
//! @tparam OutputDataType The type of the output data
//! @tparam InputGateTokenType The type of the input gate token, must inherit from DefaultInputGateToken
//! @tparam ExecuteTokenType The type of the execution token, must inherit from DefaultExecToken
//!
//! This class represents a node in an asynchronous processing pipeline that allows
//! single producer and single consumer operations. It overwrites the previous input
//! data when new data is put into the node. It has a data token which controls when the
//! data is consumeable (i.e. the input data has been updated). You are supposed to
//! use put_data() to put data into the node, and fire() to fire a data token.
//!
//! The node is designed to work with TBB (Threading Building Blocks) flow graph,
//! providing a composite node that processes input data and produces output data
//! along with associated tokens for synchronization and error handling.
//!
//! Usage example:
//! @code
//! tbb::flow::graph g;
//! struct InputData { size_t value; };
//! struct OutputData { size_t result; };
//! struct InputGateToken : public redoxi_works::async_processor::DefaultInputGateToken {
//!     size_t pushed_seq = 0;
//! };
//!
//! redoxi_works::async_processor::SingleOverwriteExecNode<InputData, OutputData, InputGateToken> node(g);
//! node.set_preserve_order(false);
//!
//! node.set_work_function([](const auto &input, auto &output) {
//!     std::get<0>(output).result = std::get<0>(input).value * 100;
//!     return 0; // Success
//! });
//!
//! node.set_output_callback([](const auto &output) {
//!     auto result = std::get<0>(output).result;
//!     auto token = std::get<1>(output);
//!     spdlog::info("Output: Result = {}, Token sequence = {}", result, token.sequence_number);
//!     return 0; // Success
//! });
//!
//! node.build();
//! @endcode
*/
template <typename InputDataType,
          typename OutputDataType,
          typename InputGateTokenType = DefaultInputGateToken,
          typename ExecuteTokenType = DefaultExecToken,
          typename = std::enable_if_t<std::is_base_of_v<DefaultInputGateToken, InputGateTokenType>>,
          typename = std::enable_if_t<std::is_base_of_v<DefaultExecToken, ExecuteTokenType>>>
class SingleOverwriteExecNode : public tbb::flow::composite_node<
                                    std::tuple<InputDataType, InputGateTokenType>,
                                    std::tuple<std::tuple<OutputDataType, InputGateTokenType>>>
{
  public:
    using InputData_t = InputDataType;
    using OutputData_t = OutputDataType;

    using InputWithTokens_t = std::tuple<InputDataType, InputGateTokenType, ExecuteTokenType>;
    using OutputWithTokens_t = std::tuple<OutputDataType, InputGateTokenType, ExecuteTokenType>;
    using OutputWithDataToken_t = std::tuple<OutputDataType, InputGateTokenType>;

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
        std::tuple<InputDataType, InputGateTokenType>,
        std::tuple<std::tuple<OutputDataType, InputGateTokenType>>>;

  public:
    //! Put data into the node, will overwrite the previous data, return true if success
    virtual bool put_data(const InputDataType &data)
    {
        if (m_input_data_node)
            return m_input_data_node->try_put(data);
        return false;
    }

    //! Try to fire the data into the processing pipeline, using the given token
    //! @return true if the token is accepted, false otherwise (previous token is not consumed).
    //! In either case, the pipeline will be notified that a new data is available, and will process it later
    bool fire(InputGateTokenType &token)
    {
        if (m_input_gate_token_limiter) {
            generate_input_gate_token(token);
            return m_input_gate_token_limiter->try_put(token);
        }
        return false;
    }

    //! Try to fire the data into the processing pipeline, return true if a new token is accepted
    bool fire()
    {
        InputGateTokenType token;
        return fire(token);
    }

  public:
    SingleOverwriteExecNode(tbb::flow::graph &g,
                            bool is_async = false,
                            bool preserve_order = true,
                            bool is_serial = false,
                            size_t execute_token_size = DEFAULT_EXECUTE_TOKEN_SIZE)
        : CompositeNode_t(g)
    {
        m_is_serial = is_serial;
        m_is_async = is_async;
        m_preserve_order = preserve_order;
        m_execute_token_size = execute_token_size;
        m_next_sequence_number = std::make_shared<std::atomic<std::size_t>>(0);
    }

    virtual ~SingleOverwriteExecNode() = default;

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

        // limit the number of input gate tokens to 1
        m_input_gate_token_limiter = std::make_shared<InputGateTokenLimiter_t>(g, 1);

        // buffer the input gate tokens
        m_input_gate_token_buf = std::make_shared<InputGateTokenBuf_t>(g);

        // overwrite the input data
        m_input_data_node = std::make_shared<InputDataNode_t>(g);

        // join the input data
        m_input_join_node = std::make_shared<InputJoinNode_t>(g);

        // reset the input gate token buffer limiter
        m_input_stamp_node = std::make_shared<InputStampNode_t>(
            g, tbb::flow::serial,
            [this](const InputWithTokens_t &input_data, typename InputStampNode_t::output_ports_type &ports) {
                // stamp the input data with the input gate token
                InputWithTokens_t stamped_data = input_data;
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
        if (m_is_async)
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
        if (m_is_async)
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
                spdlog::info("[GRAPH] Finalize node, output_data={}, input_gate_token={}", std::get<0>(output_data).result, std::get<1>(output_data).sequence_number);
                const auto &output = std::get<0>(output_data);
                const auto &data_token = std::get<1>(output_data);
                const auto &exec_token = std::get<2>(output_data);

                // output to data port
                std::get<0>(ports).try_put(std::make_tuple(output, data_token));

                // return execution token
                std::get<1>(ports).try_put(exec_token);
            });

        // build the main graph

        // create a limited buffer for input gate tokens
        // (external input) -> input_gate_token_limiter -> input_gate_token_buf
        tbb::flow::make_edge(*m_input_gate_token_limiter, *m_input_gate_token_buf);

        // (input_data, input_gate_token, exec_token) -> join_node
        tbb::flow::make_edge(*m_input_data_node, tbb::flow::input_port<0>(*m_input_join_node));
        tbb::flow::make_edge(*m_input_gate_token_buf, tbb::flow::input_port<1>(*m_input_join_node));
        tbb::flow::make_edge(*m_execute_token_buf, tbb::flow::input_port<2>(*m_input_join_node));

        // (input_data, input_gate_token, exec_token) -> stamp_node
        tbb::flow::make_edge(*m_input_join_node, *m_input_stamp_node);

        // stamp_node.port[0] -> work_node or async_work_node
        if (m_is_async)
            tbb::flow::make_edge(tbb::flow::output_port<0>(*m_input_stamp_node), *m_async_work_node);
        else
            tbb::flow::make_edge(tbb::flow::output_port<0>(*m_input_stamp_node), *m_work_node);

        // stamp_node.port[1] -> release the input gate token slot
        tbb::flow::make_edge(tbb::flow::output_port<1>(*m_input_stamp_node), m_input_gate_token_limiter->decrementer());

        // work_node -> (output_sequencer_node) -> finalize_node
        if (m_preserve_order) {
            // if preserve order is requested, work_node -> sequencer_node -> output_callback_node -> finalize_node
            assert(m_output_sequencer_node != nullptr);
            if (m_is_async) {
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
            if (m_is_async) {
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
            CompositeInputPorts_t(
                *m_input_data_node,
                *m_input_gate_token_limiter),
            CompositeOutputPorts_t(
                tbb::flow::output_port<0>(*m_finalize_node)));

        // prepare the node
        prepare();
    }

    //! Generate input gate token, created by user
    virtual void generate_input_gate_token(InputGateTokenType &token)
    {
        // sequence number is only incremented by the finalize node
        token.sequence_number = *m_next_sequence_number;
    }

    //! Set if the node is async, only callable before build()
    virtual void set_is_async(bool is_async)
    {
        assert(!is_built());
        m_is_async = is_async;
    }

    //! Check if the node is async
    virtual bool is_async() const
    {
        return m_is_async;
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

    //! Generate input gate token
    virtual InputGateTokenType generate_input_gate_token()
    {
        InputGateTokenType token;
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
        spdlog::info("[GRAPH] stamped, value={}, token={}", std::get<0>(input_data).value, std::get<1>(input_data).sequence_number);
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
            gateway.try_put(output_data);
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
            // gateway.reserve_wait();
            gateway.try_put(output_data);
            // gateway.release_wait();
        }
    }

  protected:
    // Node types

    //! used to provide input data, always overwrite the previous data
    using InputDataNode_t = tbb::flow::overwrite_node<InputDataType>;

    //! only allow one input gate token to enter at a time
    using InputGateTokenLimiter_t = tbb::flow::limiter_node<InputGateTokenType>;

    //! signals that the input data is updated and ready to be consumed
    using InputGateTokenBuf_t = tbb::flow::queue_node<InputGateTokenType>;

    //! signals that execution quota is available
    using ExecTokenBuf_t = tbb::flow::queue_node<ExecuteTokenType>;

    //! read input when required tokens are ready
    using InputJoinNode_t = tbb::flow::join_node<InputWithTokens_t, tbb::flow::reserving>;

    //! after join, stamp the input data with the input gate token
    using InputStampNode_t = tbb::flow::multifunction_node<
        InputWithTokens_t,
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

    size_t m_execute_token_size = 1;
    bool m_is_async = false;
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
    std::shared_ptr<InputGateTokenLimiter_t> m_input_gate_token_limiter;
    std::shared_ptr<InputGateTokenBuf_t> m_input_gate_token_buf;

    std::shared_ptr<InputDataNode_t> m_input_data_node;
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
