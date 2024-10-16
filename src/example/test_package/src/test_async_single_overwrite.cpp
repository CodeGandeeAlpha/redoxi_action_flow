// test the single overwrite exec node

#include <iostream>
#include <tbb/flow_graph.h>
#include <redoxi_common_cpp/async_processor/SingleOverwriteExecNode.hpp>
#include <vector>
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>

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
        std::get<0>(output).result = std::get<0>(input).value * 100;
        auto token = std::get<1>(input);
        spdlog::info("[GRAPH] Work function: Input value = {}, Output result = {}, Token sequence number = {}",
                     std::get<0>(input).value, std::get<0>(output).result, token.sequence_number);
        return 0; // Success
    });

    //! Set output callback
    node.set_output_callback([](const auto &output) {
        auto token = std::get<1>(output);
        spdlog::info("[GRAPH] Output callback: Result = {}, Token sequence number = {}", std::get<0>(output).result, token.sequence_number);
        return 0; // Success
    });

    node.set_preserve_order(false);

    //! Build the node
    node.build();

    //! Create input port
    auto &input_data_port = tbb::flow::input_port<0>(node);
    auto &input_gate_token_port = tbb::flow::input_port<1>(node);

    //! Create input data
    std::vector<InputData> input_data_vector = {{5}, {10}, {15}, {20}, {25}};

    //! Send input data
    for (const auto &input_data : input_data_vector) {
        //! Generate input gate token
        auto token = node.generate_input_gate_token();
        bool token_pushed = false;
        bool data_pushed = false;

        //! Try pushing token first, then data if token push succeeds
        while (!token_pushed || !data_pushed) {
            if (!token_pushed) {
                token_pushed = input_gate_token_port.try_put(token);
                spdlog::info("[MAIN] Attempted to push token: {}, Token value: {}", token_pushed ? "success" : "fail", token.sequence_number);
            }

            if (token_pushed && !data_pushed) {
                data_pushed = input_data_port.try_put(input_data);
                spdlog::info("[MAIN] Attempted to push data: {}", data_pushed ? "success" : "fail");
            }

            if (!token_pushed || !data_pushed) {
                //! Either token push failed or data push failed after token push succeeded
                spdlog::info("[MAIN] Push failed. Retrying to push token and data for input value {} and token sequence number {}...", input_data.value, token.sequence_number);
                spdlog::info("[MAIN] Current token sequence number: {}", token.sequence_number);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        spdlog::info("[MAIN] Sent input data: value = {}, token number = {}, token push success = {}, data push success = {}",
                     input_data.value, token.sequence_number, token_pushed, data_pushed);
        // g.wait_for_all();
    }

    //! Wait for the graph to complete
    g.wait_for_all();

    return 0;
}
