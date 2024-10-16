#pragma once
#include <redoxi_common_cpp/async_processor/common.hpp>
#include <functional>
#include <tbb/tbb.h>

namespace redoxi_works
{

namespace async_processor
{

template <typename InputDataType,
          typename OutputDataType,
          typename InputGateTokenType = DummyToken,
          typename ExecuteTokenType = DummyToken>
class SingleOverwriteExecNode : public tbb::flow::composite_node<
                                    std::tuple<InputDataType, InputGateTokenType, ExecuteTokenType>,
                                    std::tuple<OutputDataType, InputGateTokenType, ExecuteTokenType>>
{
  public:
    using InputWithTokens_t = std::tuple<InputDataType, InputGateTokenType, ExecuteTokenType>;
    using OutputWithTokens_t = std::tuple<OutputDataType, InputGateTokenType, ExecuteTokenType>;

    //! Work function type, complete version
    using WorkFunction_t = std::function<OutputWithTokens_t(const InputWithTokens_t &)>;
    //! Work function type, simple version
    using WorkFunctionSimple_t = std::function<OutputDataType(const InputDataType &)>;

    //! User output function type, complete version
    using OutputCallback_t = std::function<OutputWithTokens_t(const OutputWithTokens_t &)>;
    //! User output function type, simple version
    using OutputCallbackSimple_t = std::function<OutputDataType(const OutputDataType &)>;

    using CompositeNode_t = tbb::flow::composite_node<
        std::tuple<InputDataType, InputGateTokenType, ExecuteTokenType>,
        std::tuple<OutputDataType, InputGateTokenType, ExecuteTokenType>>;

  public:
    SingleOverwriteExecNode(tbb::flow::graph &g,
                            size_t execute_token_size = 1,
                            const WorkFunction_t &work_function = WorkFunction_t(),
                            const OutputCallback_t &output_callback = OutputCallback_t())
        : CompositeNode_t(g),
          m_work_function(work_function),
          m_output_callback(output_callback),
          m_execute_token_size(execute_token_size)
    {
        // empty
        this->graph_reference();
        this->my_graph;
    }
    ~SingleOverwriteExecNode() = default;

    virtual void build()
    {
    }

  protected:
    WorkFunction_t m_work_function;
    OutputCallback_t m_output_callback;
    size_t m_execute_token_size = 1;

    //! use shared_ptr so that you do not have to create them in constructor
    std::shared_ptr<typename tbb::flow::queue_node<ExecuteTokenType>> m_execute_token_queue;
    std::shared_ptr<typename tbb::flow::queue_node<InputGateTokenType>> m_input_gate_token_queue;
    std::shared_ptr<typename tbb::flow::overwrite_node<InputDataType>> m_input_overwrite_node;
    std::shared_ptr<typename tbb::flow::join_node<InputWithTokens_t, tbb::flow::reserving>> m_input_join_node;
    std::shared_ptr<typename tbb::flow::function_node<InputWithTokens_t, OutputWithTokens_t>> m_work_node;
    std::shared_ptr<typename tbb::flow::function_node<OutputWithTokens_t, OutputWithTokens_t>> m_output_node;
    std::shared_ptr<typename tbb::flow::function_node<OutputWithTokens_t, OutputWithTokens_t>> m_finalize_node;
};

} // namespace async_processor

} // namespace redoxi_works
