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
    bool is_async = true;
    tbb::task_arena arena(3);

    //! Define input and output structures
    struct InputData {
        size_t value;
    };

    struct OutputData {
        size_t result;
    };

    struct InputGateToken : public redoxi_works::async_processor::DefaultInputGateToken {
        size_t pushed_seq = 0;
    };

    //! Define input and output types
    using InputType = InputData;
    using OutputType = OutputData;

    //! Create a SingleOverwriteExecNode
    using Node_t = redoxi_works::async_processor::SingleOverwriteExecNode<InputType, OutputType, InputGateToken>;
    Node_t node(g);
    node.set_preserve_order(false);
    node.set_is_async(is_async);
    node.set_execute_token_size(10);
    node.set_is_serial(false);

    tbb::concurrent_bounded_queue<int> data_overwrite_tokens;
    data_overwrite_tokens.set_capacity(1);
    data_overwrite_tokens.push(0);

    //! Set work function
    auto work_function = [&](const Node_t::InputWithTokens_t &input,
                             Node_t::OutputWithTokens_t &output) -> int {
        std::get<0>(output).result = std::get<0>(input).value * 100;
        auto token = std::get<1>(input);
        uint64_t thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
        spdlog::info("[GRAPH][pid={}] Work function: Input value = {}, Output result = {}, Token sequence number = {}",
                     thread_id, std::get<0>(input).value, std::get<0>(output).result, token.sequence_number);

        // notify that a new number can be overwritten
        spdlog::info("[GRAPH][pid={}] Pushing overwrite token: {}", thread_id, token.sequence_number);
        data_overwrite_tokens.push(token.sequence_number);
        spdlog::info("[GRAPH][pid={}] Pushed overwrite token: {}", thread_id, token.sequence_number);

        // wait for a random number of ms, between 10 and 500
        // std::this_thread::sleep_for(std::chrono::milliseconds(rand() % 490 + 10));
        return 0; // Success
    };

    if (is_async) {
        node.set_work_function_async(
            [&](const auto &input, auto &output, auto &gateway) {
                // reserve the output port, so that the output callback can be executed in parallel
                spdlog::info("[GRAPH] Reserve output port, input gate token: {}", std::get<1>(input).sequence_number);
                gateway.reserve_wait();

                arena.execute([&]() {
                    auto error_code = work_function(input, output);
                    std::get<2>(output).error_code = error_code;
                    gateway.try_put(output);
                    spdlog::info("[GRAPH] Releasing output port, input gate token: {}", std::get<1>(input).sequence_number);
                    gateway.release_wait();
                });
                spdlog::info("[GRAPH] Released output port, input gate token: {}", std::get<1>(input).sequence_number);
            });
    } else {
        node.set_work_function(work_function);
    }

    //! Set output callback
    node.set_output_callback([](const auto &output) {
        auto token = std::get<1>(output);
        auto result = std::get<0>(output).result;
        //! Output callback: print result, token sequence number, and pushed_seq
        spdlog::info("[GRAPH] Output callback: Result = {}, Token sequence number = {}, Pushed sequence = {}", result, token.sequence_number, token.pushed_seq);
        if (token.pushed_seq != token.sequence_number) {
            spdlog::warn("[GRAPH] Warning: Pushed sequence ({}) is not equal to token sequence number ({})", token.pushed_seq, token.sequence_number);
            // exit(0);
        }
        return 0; // Success
    });

    //! Build the node
    spdlog::info("[GRAPH] Building node...");
    node.build();
    spdlog::info("[GRAPH] Node built");

    //! Create input port
    auto &input_data_port = tbb::flow::input_port<0>(node);
    auto &input_gate_token_port = tbb::flow::input_port<1>(node);

    //! Create input data
    std::vector<InputData> input_data_vector;
    input_data_vector.reserve(10000);
    for (int i = 0; i < 10000; ++i) {
        input_data_vector.push_back({i});
    }

    //! Send input data in a separate thread, otherwise if it blocks the main thread,
    //! the tbb scheduler will not work
    std::thread input_thread([&]() {
        for (size_t i = 0; i < input_data_vector.size(); ++i) {
            const auto &input_data = input_data_vector[i];
            //! Generate input gate token
            auto token = node.generate_input_gate_token();
            token.pushed_seq = i;
            bool token_pushed = false;
            bool data_pushed = false;

            // wait for a new number to be overwritten
            int overwrite_token = -1;
            uint64_t thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
            spdlog::info("[MAIN][pid={}] Trying to pop overwrite token...", thread_id);
            data_overwrite_tokens.pop(overwrite_token);
            spdlog::info("[MAIN][pid={}] Popped overwrite token: {}", thread_id, overwrite_token);
            // push data
            data_pushed = input_data_port.try_put(input_data);
            spdlog::info("[MAIN][pid={}] Attempted to push data: {}, value = {}", thread_id, data_pushed ? "success" : "fail", input_data.value);

            // push token
            // FIXME: if may happen that the token is consumed but you cannot push it immediately
            //        because the limiter is not decremented yet, so it will deadlock
            if (false) {
                // try until success
                while (!token_pushed) {
                    spdlog::info("[MAIN][pid={}] Attempting to push token: {}", thread_id, token.sequence_number);
                    token_pushed = input_gate_token_port.try_put(token);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } else {
                // try once only
                token_pushed = input_gate_token_port.try_put(token);
            }
            spdlog::info("[MAIN][pid={}] Attempted to push token: {}", thread_id, token_pushed ? "success" : "fail");

            //! Print the result
            spdlog::info("[MAIN][pid={}] Sent input data: value = {}, data push success = {}, token push success = {}",
                         thread_id, input_data.value, data_pushed, token_pushed);

            // g.wait_for_all();
        }
    });

    // Wait for the input thread to finish
    input_thread.join();
    g.wait_for_all();

    return 0;
}
