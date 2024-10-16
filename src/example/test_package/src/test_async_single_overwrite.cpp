// test the single overwrite exec node

#include <iostream>
#include <tbb/flow_graph.h>
#include <redoxi_common_cpp/async_processor/SingleOverwriteExecNode.hpp>
#include <vector>
#include <spdlog/spdlog.h>

int main()
{
    tbb::flow::graph g;

    //! Define input and output structures
    struct InputData {
        int value;
    };

    struct OutputData {
        int result;
    };

    //! Define input and output types
    using InputType = InputData;
    using OutputType = OutputData;

    //! Create a SingleOverwriteExecNode
    redoxi_works::async_processor::SingleOverwriteExecNode<InputType, OutputType> node(g);

    //! Set work function
    node.set_work_function([](const auto &input, auto &output) {
        std::get<0>(output).result = std::get<0>(input).value * 2;
        spdlog::info("Work function: Input value = {}, Output result = {}", std::get<0>(input).value, std::get<0>(output).result);
        return 0; // Success
    });

    //! Set output callback
    node.set_output_callback([](const auto &output) {
        spdlog::info("Output callback: Result = {}", std::get<0>(output).result);
        return 0; // Success
    });

    //! Build the node
    node.build();

    //! Create input port
    auto &input_data_port = tbb::flow::input_port<0>(node);
    auto &input_gate_token_port = tbb::flow::input_port<1>(node);

    //! Create input data
    std::vector<InputData> input_data_vector = {{5}, {10}, {15}};

    //! Send input data
    for (const auto &input_data : input_data_vector) {
        //! Generate input gate token
        auto token = node.generate_input_gate_token();
        input_data_port.try_put(input_data);
        input_gate_token_port.try_put(token);
        spdlog::info("Sent input data: value = {}", input_data.value);
    }

    //! Wait for the graph to complete
    g.wait_for_all();

    return 0;
}
