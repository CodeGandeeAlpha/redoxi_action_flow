//! Test the SingleBufferExecNode in both synchronous and asynchronous modes

#include <iostream>
#include <tbb/flow_graph.h>
#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <vector>
#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <future>

//! Define input and output structures
struct InputData {
    size_t value;
};

struct OutputData {
    size_t result;
};

//! Define input and output types
using InputType = InputData;
using OutputType = OutputData;

//! Function to run the test for both sync and async modes
void run_test(bool is_async)
{
    tbb::flow::graph g;
    tbb::task_arena arena(10);

    //! Create a SingleBufferExecNode
    using Node_t = redoxi_works::async_processor::SingleBufferExecNode<InputType, OutputType>;
    Node_t node(g);
    // node.set_is_serial(false);
    // node.set_preserve_order(false);
    node.set_is_async(is_async);
    node.set_execute_token_size(10);
    node.set_input_data_buffer_size(50);

    //! Set work function
    auto work_func = [](const Node_t::InputWithTokens_t &input,
                        Node_t::OutputWithTokens_t &output) -> int {
        //! Perform time-consuming computations
        double result = 0.0;
        for (int i = 0; i < 1000000; ++i) {
            result += std::sin(std::get<0>(input).value * i);
        }

        std::get<0>(output).result = static_cast<size_t>(std::abs(result) * 100);
        uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        spdlog::debug("[WORKER][{}] Work function: Input value = {}, Output result = {}",
                      thread_id, std::get<0>(input).value, std::get<0>(output).result);

        return 0; // Success
    };

    if (is_async) {
        node.set_work_function_async([&](const Node_t::InputWithTokens_t &input,
                                         Node_t::OutputWithTokens_t &output,
                                         typename Node_t::AsyncWorkNode_t::gateway_type &gateway) {
            gateway.reserve_wait();
            arena.execute([&]() {
                int result = work_func(input, output);
                gateway.try_put(output);
                gateway.release_wait();
            });
        });
    } else {
        node.set_work_function(work_func);
    }

    //! Set output callback
    auto output_callback = [](const Node_t::OutputWithTokens_t &output) {
        uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        spdlog::info("[WORKER][{}] Output callback: Result = {}, Sequence number = {}",
                     thread_id, std::get<0>(output).result, std::get<1>(output).sequence_number);
        return 0; //! Success
    };

    if (is_async) {
        node.set_output_callback_async([&](const Node_t::OutputWithTokens_t &output,
                                           typename Node_t::AsyncOutputCallbackNode_t::gateway_type &gateway) {
            gateway.reserve_wait();
            arena.execute([&]() {
                output_callback(output);
                gateway.try_put(output);
                gateway.release_wait();
            });
        });
    } else {
        node.set_output_callback(output_callback);
    }

    //! Build the node
    node.build();

    //! Create input data
    std::vector<InputData> input_data_vector;
    input_data_vector.reserve(100);
    for (size_t i = 1; i <= 100; ++i) {
        input_data_vector.push_back({i});
    }

    //! Send input data
    auto send_input_data = [&]() {
        uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        for (const auto &input_data : input_data_vector) {
            bool success = false;
            while (!success) {
                success = node.put_data(input_data);
                if (success) {
                    spdlog::info("[WORKER][{}] Successfully sent input data: value = {}", thread_id, input_data.value);
                } else {
                    spdlog::warn("[WORKER][{}] Failed to send input data: value = {}. Retrying in 10ms.", thread_id, input_data.value);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    };

    //! Run the test
    spdlog::info("Starting test in {} mode", is_async ? "async" : "sync");
    auto start_time = std::chrono::high_resolution_clock::now();

    std::thread input_thread(send_input_data);
    input_thread.join();
    g.wait_for_all();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    spdlog::info("Test completed in {} mode. Duration: {} ms", is_async ? "async" : "sync", duration.count());
}

int main()
{
    spdlog::set_level(spdlog::level::info);

    //! Run tests for both sync and async modes
    run_test(false); // Synchronous mode
    run_test(true);  // Asynchronous mode

    return 0;
}
