// test onetbb library, learn to use it
#include <iostream>
#include <thread>
#include <random>
#include <chrono>

#include <tbb/tbb.h>
#include <tbb/flow_graph.h>
#include <tbb/concurrent_queue.h>
#include <spdlog/spdlog.h>
using TokenType = int;

int main()
{
    // create a graph that can handle a job whenever it arrives
    tbb::flow::graph g;

    // create a concurrent bounded queue to store the tokens
    tbb::concurrent_bounded_queue<TokenType> token_provider;

    // create a concurrent queue to store the results
    tbb::concurrent_bounded_queue<std::tuple<TokenType, int>> results;
    results.set_capacity(3);

    // create a buffer node to store the tokens
    tbb::flow::buffer_node<TokenType> token_buffer(g);

    // create an input node to generate random int
    tbb::flow::input_node<int> input_node_1(g, [](tbb::flow_control &fc) -> int {
        auto v = rand() % 10;
        return v;
    });
    input_node_1.activate();

    // create an input node to generate random int
    tbb::flow::input_node<int> input_node_2(g, [](tbb::flow_control &fc) -> int {
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
                        spdlog::info("computing: {} + {}", std::get<1>(v), std::get<2>(v));
                        return std::make_tuple(std::get<0>(v), std::get<1>(v) + std::get<2>(v));
                    });
    tbb::flow::make_edge(join_input, process_sum);

    // this node buffers output and write them into the results queue
    tbb::flow::function_node<std::tuple<TokenType, int>>
        buffer_and_write(g, tbb::flow::serial,
                         [&results](const std::tuple<TokenType, int> &v) {
                             spdlog::info("result: {}, writing to output queue", std::get<1>(v));
                             results.push(v);
                             spdlog::info("output queue writing done");
                         });
    tbb::flow::make_edge(process_sum, buffer_and_write);

    // create a thread to provide tokens, every 1 second
    std::thread provider([&]() {
        for (int i = 0; i < 100; i++) {
            token_provider.push(i);
            spdlog::info("push token: {}", i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // create a thread to consume tokens
    std::thread consumer([&]() {
        tbb::task_group tg;
        while (true) {
            TokenType token;
            token_provider.pop(token);
            spdlog::info("pop token: {}", token);

            tg.run([&]() {
                auto ok = token_buffer.try_put(token);
                if (!ok) {
                    spdlog::error("failed to push token: {}", token);
                } else {
                    spdlog::info("push token to graph: {}", token);
                }
                g.wait_for_all();
            });
        }
        tg.wait();
    });

    provider.join();
    consumer.join();

    return 0;
}

int test_basic()
{
    tbb::parallel_for(0, 10, [](int i) {
        std::cout << "Hello from thread " << tbb::this_task_arena::current_thread_index() << std::endl;
    });

    return 0;
}
