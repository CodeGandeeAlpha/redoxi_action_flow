#include <tbb/tbb.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <random>
#include <spdlog/spdlog.h>
#include <fmt/format.h>
#include <fmt/color.h>

int do_works_with_arena();
int do_works_avoid_main_thread();

int main()
{
    do_works_avoid_main_thread();
    return 0;
}

//! This function demonstrates how to avoid running tasks on the main thread
//! This function creates 1000 tasks in worker threads and waits for them to complete.
//! It throws an exception if any task is running on the main thread.
int do_works_avoid_main_thread()
{
    tbb::task_group group;
    std::atomic<int> task_counter(0);
    std::thread::id main_thread_id = std::this_thread::get_id();
    std::shared_ptr<tbb::task_arena> arena;

    //! Create 1000 tasks in worker threads
    tbb::this_task_arena::isolate([&]() {
        // create arena here, and then use it to execute tasks
        arena = std::make_shared<tbb::task_arena>(4);

        for (int i = 0; i < 1000; ++i) {
            group.run([&task_counter, main_thread_id]() {
                std::thread::id current_thread_id = std::this_thread::get_id();
                if (current_thread_id == main_thread_id) {
                    throw std::runtime_error("Task is running on the main thread");
                }

                int task_id = ++task_counter;
                uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(current_thread_id));
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> dis(51, 200);
                int wait_ms = dis(gen);
                spdlog::info("[{}] Task {} {} (waiting for {} ms)", thread_id, task_id, fmt::format(fg(fmt::color::yellow), "started"), wait_ms);
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                spdlog::info("[{}] Task {} {}", thread_id, task_id, fmt::format(fg(fmt::color::green), "completed"));
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
    spdlog::info("All tasks completed, no task is running on the main thread");

    return 0;
}


int do_works_with_arena()
{
    tbb::task_arena arena(4);
    std::atomic<int> counter(0);

    //! Fire 10 jobs into the arena
    for (int i = 0; i < 10; ++i) {
        arena.enqueue([&counter] {
            int current_value = ++counter;
            uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
            spdlog::info("[{}] Job {} {}.", thread_id, current_value, fmt::format(fg(fmt::color::yellow), "started"));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            spdlog::info("[{}] Job {} {}.", thread_id, current_value, fmt::format(fg(fmt::color::green), "completed"));
        });
    }

    //! Wait for all jobs to complete
    arena.execute([] {});
    arena.execute([&] {
        uint64_t thread_id = static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        //! Indicate that we are waiting
        spdlog::info("[{}] Waiting in execute...", thread_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        spdlog::info("[{}] Hello, TBB!", thread_id);
    });

    //! Wait for all jobs to complete using sleep
    //! Wait for all jobs to complete using sleep
    spdlog::info("Waiting for all jobs to complete...");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
