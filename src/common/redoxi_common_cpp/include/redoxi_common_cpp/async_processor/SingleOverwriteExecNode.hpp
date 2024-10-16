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

// useful when development, because clangd fails to auto suggest with template parameters
// using InputDataType = int;
// using OutputDataType = int;
// using InputGateTokenType = DefaultInputGateToken;
// using ExecuteTokenType = DefaultExecToken;

template <typename InputDataType,
          typename OutputDataType,
          typename InputGateTokenType = DefaultInputGateToken,
          typename ExecuteTokenType = DefaultExecToken,
          typename = std::enable_if_t<std::is_base_of_v<DefaultInputGateToken, InputGateTokenType>>,
          typename = std::enable_if_t<std::is_base_of_v<DefaultExecToken, ExecuteTokenType>>>
class SingleOverwriteExecNode : public tbb::flow::composite_node<
                                    std::tuple<InputDataType, InputGateTokenType>,
                                    std::tuple<OutputDataType, InputGateTokenType>>
{
  public:
    using InputData_t = InputDataType;
    using OutputData_t = OutputDataType;
    using InputWithTokens_t = std::tuple<InputDataType, InputGateTokenType, ExecuteTokenType>;
    using OutputWithTokens_t = std::tuple<OutputDataType, InputGateTokenType, ExecuteTokenType>;

    /**
     * @brief Work function type, update output by input,
     *        (input, output) -> error_code, error_code = 0 means success
     * @note The work function is executed in parallel with other work nodes,
     *       so you HAVE TO care about the thread safety.
     */
    using WorkFunction_t = std::function<int(const InputWithTokens_t &, OutputWithTokens_t &)>;

    /**
     * @brief User callback over the output data,
     *        (output_data) -> error_code, error_code = 0 means success
     * @note The output callback is executed in parallel with other output callback nodes,
     *       so you HAVE TO care about the thread safety.
     */
    using OutputCallback_t = std::function<int(const OutputWithTokens_t &)>;

    using CompositeNode_t = tbb::flow::composite_node<
        std::tuple<InputDataType, InputGateTokenType>,
        std::tuple<OutputDataType, InputGateTokenType>>;

  public:
    SingleOverwriteExecNode(tbb::flow::graph &g,
                            const WorkFunction_t &work_function = WorkFunction_t(),
                            const OutputCallback_t &output_callback = OutputCallback_t(),
                            size_t execute_token_size = 1,
                            bool preserve_order = true,
                            bool is_serial = false)
        : CompositeNode_t(g),
          m_work_function(work_function),
          m_output_callback(output_callback),
          m_execute_token_size(execute_token_size),
          m_preserve_order(preserve_order),
          m_is_serial(is_serial)
    {
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
        m_execute_token_buf = std::make_shared<tbb::flow::queue_node<ExecuteTokenType>>(g);

        // limit the number of input gate tokens to 1
        m_input_gate_token_limiter = std::make_shared<tbb::flow::limiter_node<InputGateTokenType>>(g, 1);

        // buffer the input gate tokens
        m_input_gate_token_buf = std::make_shared<tbb::flow::queue_node<InputGateTokenType>>(g);

        // overwrite the input data
        m_input_overwrite_node = std::make_shared<tbb::flow::overwrite_node<InputDataType>>(g);

        // join the input data
        m_input_join_node = std::make_shared<tbb::flow::join_node<InputWithTokens_t, tbb::flow::reserving>>(g);

        // broadcast the input data
        m_input_broadcast_node = std::make_shared<tbb::flow::broadcast_node<InputWithTokens_t>>(g);

        // reset the input gate token buffer limiter
        m_input_gate_token_buf_reset_node = std::make_shared<tbb::flow::function_node<InputWithTokens_t, tbb::flow::continue_msg>>(
            g, concurrency_type, [](const InputWithTokens_t &) {
                return tbb::flow::continue_msg();
            });

        // sequencer node
        m_output_sequencer_node = nullptr;
        if (m_preserve_order)
            m_output_sequencer_node = std::make_shared<tbb::flow::sequencer_node<OutputWithTokens_t>>(
                g, [](const OutputWithTokens_t &output_data) {
                    return std::get<1>(output_data).sequence_number;
                });

        // work node
        m_work_node = std::make_shared<tbb::flow::function_node<InputWithTokens_t, OutputWithTokens_t>>(
            g, concurrency_type, [this](const InputWithTokens_t &input_data) {
                return this->_nodefunc_work_output(input_data);
            });

        // finalize node
        m_finalize_node = std::make_shared<tbb::flow::function_node<OutputWithTokens_t, OutputWithTokens_t>>(
            g, concurrency_type,
            [this](const OutputWithTokens_t &output_data) {
                return this->_nodefunc_finalize(output_data);
            });

        // output split node
        m_output_split_node = std::make_shared<tbb::flow::split_node<OutputWithTokens_t>>(g);

        // build the main graph
        tbb::flow::make_edge(*m_input_gate_token_limiter, *m_input_gate_token_buf);
        tbb::flow::make_edge(*m_input_overwrite_node, tbb::flow::input_port<0>(*m_input_join_node));
        tbb::flow::make_edge(*m_input_gate_token_buf, tbb::flow::input_port<1>(*m_input_join_node));
        tbb::flow::make_edge(*m_execute_token_buf, tbb::flow::input_port<2>(*m_input_join_node));
        tbb::flow::make_edge(*m_input_join_node, *m_input_broadcast_node);
        tbb::flow::make_edge(*m_input_broadcast_node, *m_work_node);

        if (m_preserve_order) {
            // if preserve order is requested, we need to sequence the output
            assert(m_output_sequencer_node != nullptr);
            tbb::flow::make_edge(*m_work_node, *m_output_sequencer_node);
            tbb::flow::make_edge(*m_output_sequencer_node, *m_finalize_node);
        } else {
            tbb::flow::make_edge(*m_work_node, *m_finalize_node);
        }

        // side graph: reset input gate token limit, to allow new tokens to enter
        tbb::flow::make_edge(*m_input_broadcast_node, *m_input_gate_token_buf_reset_node);
        tbb::flow::make_edge(*m_input_gate_token_buf_reset_node, m_input_gate_token_limiter->decrementer());

        // side graph: return exec token to the buffer, so that it can be reused
        tbb::flow::make_edge(*m_finalize_node, *m_output_split_node);
        tbb::flow::make_edge(tbb::flow::output_port<2>(*m_output_split_node), *m_execute_token_buf);

        // define input ports
        using CompositeInputPorts_t = typename CompositeNode_t::input_ports_type;
        using CompositeOutputPorts_t = typename CompositeNode_t::output_ports_type;
        this->set_external_ports(
            CompositeInputPorts_t(
                *m_input_overwrite_node,
                *m_input_gate_token_buf),
            CompositeOutputPorts_t(
                tbb::flow::output_port<0>(*m_output_split_node),
                tbb::flow::output_port<1>(*m_output_split_node)));

        // prepare the node
        _reset();
    }

    //! Generate input gate token, created by user
    virtual void generate_input_gate_token(InputGateTokenType &token)
    {
        // sequence number is only incremented by the finalize node
        token.sequence_number = *m_next_sequence_number;
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
        m_work_function = work_function;
    }

    //! Set the output callback, must be done before build()
    virtual void set_output_callback(const OutputCallback_t &output_callback)
    {
        m_output_callback = output_callback;
    }

  protected:
    //! Reset the node and prepare tokens
    virtual void _reset()
    {
        *m_next_sequence_number = 0;

        // reset the input gate token buffer
        if (m_input_gate_token_buf)
            m_input_gate_token_buf->reset();

        // reset the execute token buffer
        if (m_execute_token_buf) {
            m_execute_token_buf->reset();
            for (size_t i = 0; i < m_execute_token_size; i++)
                while (!m_execute_token_buf->try_put(ExecuteTokenType()))
                    ;
        }
    }

    //! Reset the node
    virtual void reset_node(tbb::flow::reset_flags flags) override
    {
        CompositeNode_t::reset_node(flags);
        _reset();
    }

    //! Function to process work output
    OutputWithTokens_t _nodefunc_work_output(const InputWithTokens_t &input_data)
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

    //! Function to call user output callback
    OutputWithTokens_t _nodefunc_finalize(const OutputWithTokens_t &output_data)
    {
        OutputWithTokens_t _output = output_data;
        auto &exec_token = std::get<2>(_output);
        if (m_output_callback)
            exec_token.error_code = m_output_callback(_output);

        // gate token consumed, increase the sequence number
        (*m_next_sequence_number)++;

        return _output;
    }

  protected:
    WorkFunction_t m_work_function;
    OutputCallback_t m_output_callback;
    size_t m_execute_token_size = 1;

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
    std::shared_ptr<
        typename tbb::flow::queue_node<ExecuteTokenType>>
        m_execute_token_buf;
    std::shared_ptr<
        typename tbb::flow::limiter_node<InputGateTokenType>>
        m_input_gate_token_limiter;
    std::shared_ptr<
        typename tbb::flow::queue_node<InputGateTokenType>>
        m_input_gate_token_buf;
    std::shared_ptr<
        typename tbb::flow::overwrite_node<InputDataType>>
        m_input_overwrite_node;
    std::shared_ptr<
        typename tbb::flow::join_node<InputWithTokens_t, tbb::flow::reserving>>
        m_input_join_node;
    std::shared_ptr<
        typename tbb::flow::broadcast_node<InputWithTokens_t>>
        m_input_broadcast_node;
    std::shared_ptr<
        typename tbb::flow::function_node<InputWithTokens_t>>
        m_input_gate_token_buf_reset_node;
    std::shared_ptr<
        typename tbb::flow::function_node<InputWithTokens_t, OutputWithTokens_t>>
        m_work_node;
    std::shared_ptr<
        typename tbb::flow::sequencer_node<OutputWithTokens_t>>
        m_output_sequencer_node;
    std::shared_ptr<
        typename tbb::flow::function_node<OutputWithTokens_t, OutputWithTokens_t>>
        m_finalize_node;
    std::shared_ptr<
        typename tbb::flow::split_node<OutputWithTokens_t>>
        m_output_split_node;
};

} // namespace async_processor

} // namespace redoxi_works
