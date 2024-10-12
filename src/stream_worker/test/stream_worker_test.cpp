#include <spdlog/spdlog.h>
#include <stream_worker/stream_worker.hpp>

int main()
{
    spdlog::info("Hello, Stream Worker!");
    int x = 1;
    spdlog::info("x: {}", x);
    return 0;
}