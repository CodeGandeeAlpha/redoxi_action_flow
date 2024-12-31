#include <test_package/_pch.hpp>

#include <tbb/tbb.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <random>
#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/color.h>

int do_works_with_arena(int num_worker_threads, int num_tasks);
int do_works_avoid_main_thread(int num_worker_threads, int num_tasks);
int do_works_originated_from_custom_thread(int num_worker_threads, int num_tasks);
int do_works_compare_strategy(int num_worker_threads, int num_tasks);
int will_arena_isolate_block_calling_thread();

int main()
{
    // do_works_originated_from_custom_thread(2, 100);
    // do_works_compare_strategy(3, 100);
    will_arena_isolate_block_calling_thread();
    return 0;
}

//! Test if tbb::this_task_arena::isolate() will block the calling thread
int will_arena_isolate_block_calling_thread()
{
    spdlog::info("Testing if tbb::this_task_arena::isolate() will block the calling thread");
    //! Print main thread id
    uint64_t main_thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    spdlog::info("Main thread ID: {}", main_thread_id);

    std::atomic<bool> task_started(false);
    std::atomic<bool> task_finished(false);

    //! Test tbb::this_task_arena::isolate()
    spdlog::info("Testing tbb::this_task_arena::isolate()");
    tbb::this_task_arena::isolate([&]() {
        uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        spdlog::info("Starting long-running task on thread {}", thread_id);
        task_started = true;

        //! Simulate a long-running task
        std::this_thread::sleep_for(std::chrono::seconds(3));

        task_finished = true;
        spdlog::info("Long-running task finished on thread {}", thread_id);
    });

    if (task_finished) {
        spdlog::info("Task finished, the main thread is blocked by tbb::this_task_arena::isolate()");
    } else {
        spdlog::info("Task is not finished, the main thread is not blocked by tbb::this_task_arena::isolate()");
    }

    //! Test custom task_arena's execute()
    spdlog::info("Testing custom task_arena's execute()");
    task_started = false;
    task_finished = false;

    tbb::task_arena arena(2);
    arena.execute([&]() {
        uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        spdlog::info("Starting long-running task on thread {}", thread_id);
        task_started = true;

        //! Simulate a long-running task
        std::this_thread::sleep_for(std::chrono::seconds(3));

        task_finished = true;
        spdlog::info("Long-running task finished on thread {}", thread_id);
    });

    if (task_finished) {
        spdlog::info("Task finished, the main thread is blocked by custom task_arena's execute()");
    } else {
        spdlog::info("Task is not finished, the main thread is not blocked by custom task_arena's execute()");
    }

    return 0;
}


int do_works_compare_strategy(int num_worker_threads, int num_tasks)
{
    std::thread::id main_thread_id = std::this_thread::get_id();
    std::atomic<int> task_counter(0);

    auto execute_task = [&main_thread_id, &task_counter](const std::string &strategy) {
        if (std::this_thread::get_id() == main_thread_id) {
            spdlog::error("[{}] Task is running on the main thread!", strategy);
            throw std::runtime_error("Task is running on the main thread");
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(20, 50);
        int wait_ms = dis(gen);
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    };

    auto run_strategy = [&](const std::string &strategy, const std::function<void()> &strategy_func) {
        spdlog::info("Starting strategy: {}", strategy);
        task_counter.store(0);
        try {
            strategy_func();
        } catch (const std::exception &e) {
            spdlog::error("Exception in strategy {}: {}", strategy, e.what());
        }
        spdlog::info("Completed strategy: {}", strategy);
    };

    //! Strategy 1: Create task group in main thread, submit tasks to the group and wait
    run_strategy("Strategy 1", [&]() {
        spdlog::info("Strategy 1: Creating task group in main thread");
        tbb::task_group group;
        for (int i = 0; i < num_tasks; ++i) {
            group.run([&]() { execute_task("Strategy 1"); });
        }
        spdlog::info("Strategy 1: Waiting for all tasks to complete");
        group.wait();
        spdlog::info("Strategy 1: All tasks completed");
    });

    //! Strategy 2: Create a task arena and task group in main thread, use task arena's execute to add tasks into group, and wait the group in main thread
    run_strategy("Strategy 2", [&]() {
        spdlog::info("Strategy 2: Creating task arena with {} threads", num_worker_threads);
        tbb::task_arena arena(num_worker_threads);
        spdlog::info("Strategy 2: Creating task group");
        tbb::task_group group;
        spdlog::info("Strategy 2: Executing task submission in arena");
        arena.execute([&]() {
            for (int i = 0; i < num_tasks; ++i) {
                group.run([&]() { execute_task("Strategy 2"); });
            }
        });
        spdlog::info("Strategy 2: Waiting for all tasks to complete in main thread");
        group.wait();
        spdlog::info("Strategy 2: All tasks completed");
    });

    //! Strategy 3: Like Strategy 2, but wait the group in task arena's execute
    run_strategy("Strategy 3", [&]() {
        spdlog::info("Strategy 3: Creating task arena with {} threads", num_worker_threads);
        tbb::task_arena arena(num_worker_threads);
        spdlog::info("Strategy 3: Creating task group");
        tbb::task_group group;
        spdlog::info("Strategy 3: Executing task submission and wait in arena");
        arena.execute([&]() {
            for (int i = 0; i < num_tasks; ++i) {
                group.run([&]() { execute_task("Strategy 3"); });
            }
            spdlog::info("Strategy 3: Waiting for all tasks to complete within arena");
            group.wait();
        });
        spdlog::info("Strategy 3: All tasks completed");
    });

    //! Strategy 4: Create a my_worker_thread in main thread, and do the above tests inside the my_worker_thread
    run_strategy("Strategy 4", [&]() {
        spdlog::info("Strategy 4: Creating worker thread");
        std::thread my_worker_thread([&]() {
            spdlog::info("Strategy 4: Worker thread started");

            //! Strategy 4a: Create task group in worker thread
            spdlog::info("Strategy 4a: Creating task group in worker thread");
            tbb::task_group group_a;
            for (int i = 0; i < num_tasks; ++i) {
                group_a.run([&]() { execute_task("Strategy 4a"); });
            }
            spdlog::info("Strategy 4a: Waiting for all tasks to complete");
            group_a.wait();
            spdlog::info("Strategy 4a: All tasks completed");

            //! Strategy 4b: Create task arena and group in worker thread, execute in arena
            spdlog::info("Strategy 4b: Creating task arena with {} threads", num_worker_threads);
            tbb::task_arena arena_b(num_worker_threads);
            spdlog::info("Strategy 4b: Creating task group");
            tbb::task_group group_b;
            spdlog::info("Strategy 4b: Executing task submission in arena");
            arena_b.execute([&]() {
                for (int i = 0; i < num_tasks; ++i) {
                    group_b.run([&]() { execute_task("Strategy 4b"); });
                }
            });
            spdlog::info("Strategy 4b: Waiting for all tasks to complete in worker thread");
            group_b.wait();
            spdlog::info("Strategy 4b: All tasks completed");

            //! Strategy 4c: Create task arena and group in worker thread, execute and wait in arena
            spdlog::info("Strategy 4c: Creating task arena with {} threads", num_worker_threads);
            tbb::task_arena arena_c(num_worker_threads);
            spdlog::info("Strategy 4c: Creating task group");
            tbb::task_group group_c;
            spdlog::info("Strategy 4c: Executing task submission and wait in arena");
            arena_c.execute([&]() {
                for (int i = 0; i < num_tasks; ++i) {
                    group_c.run([&]() { execute_task("Strategy 4c"); });
                }
                spdlog::info("Strategy 4c: Waiting for all tasks to complete within arena");
                group_c.wait();
            });
            spdlog::info("Strategy 4c: All tasks completed");
        });
        spdlog::info("Strategy 4: Joining worker thread");
        my_worker_thread.join();
        spdlog::info("Strategy 4: Worker thread joined");
    });

    //! Conclusion
    spdlog::info("Conclusion:");
    spdlog::info("Strategies that can avoid task execution in the main thread:");
    spdlog::info("- Strategy 2: Submit tasks to task group inside arena execute, and wait the task group in main thread");
    spdlog::info("- Strategy 4: Custom worker thread, do any way you want, it never touches the main thread");
    spdlog::info("");
    spdlog::info("Strategies that do not work (may execute tasks in the main thread):");
    spdlog::info("- Strategy 1: Submit tasks to task group inside main thread and wait in main thread, without task arena");
    spdlog::info("- Strategy 3: Submit tasks to task group inside task arena's execute, and wait task group in arena execute");
    spdlog::info("");
    spdlog::info("The most effective strategy is Strategy 4, which involves:");
    spdlog::info("• Creating a custom worker thread");
    spdlog::info("• Performing all task management within the worker thread");
    spdlog::info("• Using task arenas and groups within the worker thread");
    spdlog::info("• Joining the worker thread from the main thread");
    spdlog::info("");
    spdlog::info("This approach ensures complete isolation of task execution from the main thread,");
    spdlog::info("providing the most robust solution for avoiding main thread involvement in task processing.");

    return 0;
}


int do_works_originated_from_custom_thread(int num_worker_threads, int num_tasks)
{
    //! Save the main thread id
    std::thread::id main_thread_id = std::this_thread::get_id();
    std::atomic<int> task_counter(0);

    //! Create a worker thread
    std::thread my_worker_thread([&]() {
        bool use_separate_arena = true; // Switch to control whether to use a separate task arena
        tbb::task_group group;
        std::unique_ptr<tbb::task_arena> arena;

        if (use_separate_arena) {
            arena = std::make_unique<tbb::task_arena>(num_worker_threads, 0);
        }

        //! Save the worker thread id
        std::thread::id worker_thread_id = std::this_thread::get_id();

        auto execute_tasks = [&]() {
            //! Submit num_tasks tasks
            for (int i = 0; i < num_tasks; ++i) {
                auto task = [&, worker_thread_id]() {
                    //! Check if the task is running on the main thread
                    if (std::this_thread::get_id() == main_thread_id) {
                        throw std::runtime_error("Task is running on the main thread");
                    }

                    int task_id = ++task_counter;
                    std::thread::id current_thread_id = std::this_thread::get_id();
                    uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(current_thread_id));

                    //! Generate random wait time between 50 and 100 ms
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(50, 100);
                    int wait_ms = dis(gen);

                    //! Check if the task is running on the worker thread
                    bool is_worker_thread = (current_thread_id == worker_thread_id);
                    auto thread_color = is_worker_thread ? fmt::color::magenta : fmt::color::green;

                    spdlog::info("[thread_id={}]{}Task {} {} (waiting for {} ms)", thread_id,
                                 fmt::format(fg(thread_color), "[task]"),
                                 task_id, fmt::format(fg(fmt::color::yellow), "started"), wait_ms);
                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                    spdlog::info("[thread_id={}]{}Task {} {}", thread_id,
                                 fmt::format(fg(thread_color), "[task]"),
                                 task_id, fmt::format(fg(fmt::color::green), "completed"));
                };

                if (use_separate_arena) {
                    arena->execute([&group, &task]() {
                        group.run(task);
                    });
                } else {
                    group.run(task);
                }
            }
        };

        if (use_separate_arena) {
            arena->execute(execute_tasks);
        } else {
            execute_tasks();
        }

        spdlog::info("[thread_id={}]{}Waiting for tasks to complete...",
                     static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                     fmt::format(fg(fmt::color::yellow), "[worker]"));
        group.wait();
        spdlog::info("[thread_id={}]{}All tasks completed.",
                     static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                     fmt::format(fg(fmt::color::yellow), "[worker]"));
    });

    //! Wait for the worker thread to finish
    my_worker_thread.join();

    spdlog::info("[thread_id={}]{}All tasks completed successfully",
                 static_cast<uint64_t>(std::hash<std::thread::id>{}(main_thread_id)),
                 fmt::format(fg(fmt::color::red), "[main]"));
    return 0;
}


//! This function demonstrates how to avoid running tasks on the main thread
/*!
 * This function creates a specified number of tasks in a given number of worker threads and waits for them to complete.
 * It throws an exception if any task is running on the main thread.
 *
 * @param num_worker_threads The number of worker threads to use in the task arena
 * @param num_tasks The number of tasks to create and execute
 * @return int Returns 0 on successful completion
 */
int do_works_avoid_main_thread(int num_worker_threads, int num_tasks)
{
    // we only use this number of worker threads in the arena
    // you can see that in thread id output
    const int NUM_ARENA_THREADS = num_worker_threads;
    const int NUM_TASKS = num_tasks;
    tbb::task_group group;
    std::atomic<int> task_counter(0);
    std::thread::id main_thread_id = std::this_thread::get_id();
    std::shared_ptr<tbb::task_arena> arena;

    //! Create 1000 tasks in worker threads
    tbb::this_task_arena::isolate([&]() {
        // create arena here, and then use it to execute tasks
        // 0 means the arena will also uses the calling thread as its own worker thread
        // because we are already in isolate mode, so we don't need to worry about taking over the main thread
        arena = std::make_shared<tbb::task_arena>(NUM_ARENA_THREADS, 0);
        spdlog::info("Isolation mode, current thread id: {}", static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())));

        for (int i = 0; i < NUM_TASKS; ++i) {
            arena->execute([&group, &task_counter, main_thread_id]() {
                group.run([&task_counter, main_thread_id]() {
                    std::thread::id current_thread_id = std::this_thread::get_id();
                    if (current_thread_id == main_thread_id) {
                        throw std::runtime_error("Task is running on the main thread");
                    }

                    int task_id = ++task_counter;
                    uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(current_thread_id));
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<> dis(20, 50);
                    int wait_ms = dis(gen);
                    spdlog::info("[thread_id={}] Task {} {} (waiting for {} ms)", thread_id, task_id, fmt::format(fg(fmt::color::yellow), "started"), wait_ms);
                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                    spdlog::info("[thread_id={}] Task {} {}", thread_id, task_id, fmt::format(fg(fmt::color::green), "completed"));
                });
            });
        }
    });

    //! Wait for all tasks to complete
    spdlog::info("Main thread waiting for all tasks to complete...");
    tbb::this_task_arena::isolate([&]() {
        group.wait();
    });
    // arena.execute([&group]() {
    //     group.wait();
    // });
    spdlog::info("All tasks completed, no task ever executed on the main thread");

    return 0;
}

/**
 * @brief This function demonstrates how to use tbb::task_arena to execute tasks in parallel
 * @details This function creates 10 jobs in the arena and waits for them to complete.
 *          It also demonstrates how to wait for all jobs to complete using sleep.
 */
int do_works_with_arena(int num_worker_threads, int num_tasks)
{
    std::shared_ptr<tbb::task_arena> arena;
    std::shared_ptr<tbb::task_group> group;
    std::atomic<int> counter(0);
    bool wait_in_arena = true;
    arena = std::make_shared<tbb::task_arena>(num_worker_threads);

    //! Fire num_tasks jobs into the arena
    std::thread::id main_thread_id = std::this_thread::get_id();
    arena->execute([&]() {
        group = std::make_shared<tbb::task_group>();
        for (int i = 0; i < num_tasks; ++i) {
            group->run([&counter, main_thread_id] {
                //! Assert that the task is not run in the main thread
                if (std::this_thread::get_id() == main_thread_id) {
                    throw std::runtime_error("Task is running on the main thread");
                }

                int current_value = ++counter;
                uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
                spdlog::info("[thread_id={}] Job {} {}.", thread_id, current_value, fmt::format(fg(fmt::color::yellow), "started"));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                spdlog::info("[thread_id={}] Job {} {}.", thread_id, current_value, fmt::format(fg(fmt::color::green), "completed"));
            });
        }
    });

    //! Wait for all jobs to complete
    if (wait_in_arena) {
        spdlog::info("[thread_id={}] Waiting for tasks to complete in arena...", static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())));
        arena->execute([&] {
            group->wait();
            spdlog::info("[thread_id={}] All tasks completed in arena.", static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())));
        });
    } else {
        uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        spdlog::info("[thread_id={}] Waiting for tasks to complete...", thread_id);
        group->wait();
        spdlog::info("[thread_id={}] All tasks completed.", thread_id);
    }

    return 0;
}
