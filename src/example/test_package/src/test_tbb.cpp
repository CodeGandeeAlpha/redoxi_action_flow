// test onetbb library, learn to use it
#include <iostream>
#include <thread>
#include <random>
#include <chrono>

#include <tbb/tbb.h>
#include <tbb/flow_graph.h>
#include <tbb/concurrent_queue.h>
#include <spdlog/spdlog.h>
#include <tuple>

#include "redoxi_common_cpp/async_processor/SingleOverwriteExecNode.hpp"


using TokenType = int;
int test_token_system();

int main()
{
    tbb::global_control global_limit(tbb::global_control::max_allowed_parallelism, 10);
    using AsyncProcessNode = tbb::flow::async_node<std::tuple<int, int>, int>;
    const int NUM_DATA_GENERATED = 100;

    struct AsyncTask {
        AsyncProcessNode::input_type data;
        AsyncProcessNode::gateway_type *gateway = nullptr;

        AsyncTask() = default;
        AsyncTask(AsyncProcessNode::input_type input_data, AsyncProcessNode::gateway_type *input_gateway)
            : data(input_data), gateway(input_gateway)
        {
        }
    };

    // test async node
    tbb::flow::graph g;

    // just create this to compile
    redoxi_works::async_processor::SingleOverwriteExecNode<int, int> my_composite_node(g);

    // create a concurrent queue to store the data
    tbb::concurrent_bounded_queue<std::shared_ptr<AsyncTask>> ext_data_queue;
    tbb::concurrent_bounded_queue<int> sync_output_queue;

    // create a thread that takes data from ext_data_queue, process them and put them into async_process_gateway
    std::thread async_process_thread([&]() {
        while (true) {
            std::shared_ptr<AsyncTask> task_ptr;
            ext_data_queue.pop(task_ptr);
            if (task_ptr) {
                // spdlog::info("[external thread] reserve wait");
                task_ptr->gateway->reserve_wait();
            }
            int data_1 = std::get<0>(task_ptr->data);
            int data_2 = std::get<1>(task_ptr->data);
            // spdlog::info("[external thread] get data: {}, {}", data_1, data_2);
            auto v = data_1 + data_2;
            if (task_ptr) {
                // spdlog::info("[external thread] put data: {}", v);
                task_ptr->gateway->try_put(v);
                // spdlog::info("[external thread] release wait");
                task_ptr->gateway->release_wait();
            }
            // Sleep for a random number of milliseconds
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(1, 1000); // Random sleep between 1 and 1000 ms
            int sleep_duration = dis(gen);
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_duration));
        }
    });

    // create a input node that generate random int
    tbb::flow::input_node<int> data_source_1(g, [](tbb::flow_control &fc) -> int {
        static int num_generated = 0;
        if (num_generated < NUM_DATA_GENERATED) {
            num_generated++;
            return num_generated * 10;
        }
        fc.stop();
        return -1000;
    });

    tbb::flow::input_node<int> data_source_2(g, [](tbb::flow_control &fc) -> int {
        static int num_generated = 0;
        if (num_generated < NUM_DATA_GENERATED) {
            num_generated++;
            return num_generated * 10000;
        }
        fc.stop();
        return -1000;
    });

    // create a join node to read two items from the input node
    tbb::flow::join_node<std::tuple<int, int>> multi_reader(g);
    tbb::flow::make_edge(data_source_1, tbb::flow::input_port<0>(multi_reader));
    tbb::flow::make_edge(data_source_2, tbb::flow::input_port<1>(multi_reader));

    // create a function node to add the two input data
    tbb::flow::function_node<std::tuple<int, int>, int> add_data(g, tbb::flow::unlimited, [&](const std::tuple<int, int> &v) {
        auto s = std::get<0>(v) + std::get<1>(v);
        sync_output_queue.push(s);
        return s;
    });
    tbb::flow::make_edge(multi_reader, add_data);

    // create an async node to process the data
    AsyncProcessNode async_process(
        g, tbb::flow::unlimited,
        [&](const AsyncProcessNode::input_type &v, AsyncProcessNode::gateway_type &gateway) {
            int64_t thread_id = std::hash<std::thread::id>{}(std::this_thread::get_id());
            auto a = std::get<0>(v);
            auto b = std::get<1>(v);

            // create a busy spin for random number of ms between 500~1000
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(500, 1000);
            int sleep_duration = dis(gen);

            spdlog::info("[async node {}] sleep for {} ms", thread_id, sleep_duration);
            auto start_time = std::chrono::steady_clock::now();
            while (true) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
                if (elapsed.count() >= sleep_duration) {
                    break;
                }
                // Busy-wait
            }

            spdlog::info("[async node {}] get data: a={}, b={}", thread_id, a, b);

            std::shared_ptr<AsyncTask> task = std::make_shared<AsyncTask>(std::make_tuple(a, b), &gateway);
            ext_data_queue.push(task);
        });
    // tbb::flow::make_edge(multi_reader, async_process);

    // create a output node to print the data
    tbb::flow::function_node<int> data_sink(g, tbb::flow::serial, [](int v) {
        spdlog::info("[output node] get data: {}", v);
    });
    // tbb::flow::make_edge(async_process, data_sink);

    tbb::flow::make_edge(add_data, data_sink);

    data_source_1.activate();
    data_source_2.activate();
    g.wait_for_all();

    // iterate through the sync_output_queue and print the data
    int v;
    while (sync_output_queue.try_pop(v)) {
        spdlog::info("[main thread] get data: {}", v);
    }

    async_process_thread.join();

    return 0;
}

int test_token_system()
{
    // create a graph that can handle a job whenever it arrives
    tbb::flow::graph g;

    // create a concurrent bounded queue to store the tokens
    tbb::concurrent_bounded_queue<TokenType> token_provider;

    // create a concurrent queue to store the results
    tbb::concurrent_bounded_queue<std::tuple<TokenType, int>> results;
    results.set_capacity(2);

    // create a buffer node to store the tokens
    tbb::flow::buffer_node<TokenType> token_buffer(g);

    // create an input node to generate random int
    tbb::flow::input_node<int> input_node_1(g, [](tbb::flow_control &) -> int {
        auto v = rand() % 10;
        return v;
    });
    input_node_1.activate();

    // create an input node to generate random int
    tbb::flow::input_node<int> input_node_2(g, [](tbb::flow_control &) -> int {
        auto v = rand() % 1000;
        return v;
    });
    input_node_2.activate();
    // create a join node to merge two input nodes and the token buffer
    tbb::flow::join_node<std::tuple<TokenType, int, int>> join_input(g);
    tbb::flow::make_edge(token_buffer, tbb::flow::input_port<0>(join_input));
    tbb::flow::make_edge(input_node_1, tbb::flow::input_port<1>(join_input));
    tbb::flow::make_edge(input_node_2, tbb::flow::input_port<2>(join_input));

    // create a function node to process the tokens
    tbb::flow::function_node<std::tuple<TokenType, int, int>, std::tuple<TokenType, int>>
        process_sum(g, tbb::flow::unlimited,
                    [](const std::tuple<TokenType, int, int> &v) -> std::tuple<TokenType, int> {
                        spdlog::info("task token {}, computing {}+{}", std::get<0>(v), std::get<1>(v), std::get<2>(v));
                        return std::make_tuple(std::get<0>(v), std::get<1>(v) + std::get<2>(v));
                    });
    tbb::flow::make_edge(join_input, process_sum);

    // this node buffers output and write them into the results queue
    tbb::flow::function_node<std::tuple<TokenType, int>, tbb::flow::continue_msg, tbb::flow::rejecting>
        buffer_and_write(g, tbb::flow::serial,
                         [&results](const std::tuple<TokenType, int> &v) {
                             //  spdlog::info("result: {}, writing to output queue", std::get<1>(v));
                             results.push(v);
                             //  spdlog::info("output queue writing done");
                             return tbb::flow::continue_msg();
                         });
    tbb::flow::make_edge(process_sum, buffer_and_write);

    // create a thread to provide tokens, every 1 second
    std::thread provider([&]() {
        for (int i = 0; i < 100; i++) {
            token_provider.push(i);
            // spdlog::info("push token: {}", i);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    // create a thread to pull results from the results queue
    std::thread puller([&]() {
        while (true) {
            std::tuple<TokenType, int> result;
            results.pop(result);
            spdlog::info("pull result: token={}, value={}", std::get<0>(result), std::get<1>(result));
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });

    // create a thread to consume tokens
    std::thread consumer([&]() {
        while (true) {
            TokenType token;
            token_provider.pop(token);
            spdlog::info("consume token: {}", token);
            token_buffer.try_put(token);
        }
        tbb::task_arena an{8};
        an.execute([&]() {
            g.wait_for_all();
        });
    });

    provider.join();
    consumer.join();
    puller.join();

    return 0;
}