#include <spdlog/spdlog.h>
#include <vector>
#include <thread>
#include <boost/process.hpp>
#include <opencv2/opencv.hpp>

namespace bp = boost::process;

// const std::string video_url = "udp://localhost:5555/live";
const std::string video_url = "tcp://localhost:5555/live?listen";
const std::string ffmpeg_exec = "/usr/bin/ffmpeg";
const std::vector<std::string> ffmpeg_args = {
    "-i", video_url, "-f", "rawvideo",
    "-fflags", "nobuffer", "-flags", "low_delay",
    "-pix_fmt", "bgr24", "pipe:"};
const int frame_width = 1920;
const int frame_height = 1080;
const int frame_channels = 3;

int main(int argc, char **argv)
{
    bp::ipstream data_pipe;
    bp::ipstream error_pipe;

    // Create a shared frame buffer
    cv::Mat frame(frame_height, frame_width, CV_8UC(frame_channels));
    const size_t frame_size = frame.total() * frame.elemSize();
    bool new_frame_available = false;
    bool should_exit = false;
    std::mutex frame_mutex;
    std::condition_variable frame_cv;

    // start logging thread
    std::thread logging_thread([&]() {
        std::string line;
        while (std::getline(error_pipe, line)) {
            spdlog::info("ffmpeg log: {}", line);
        }
    });
    logging_thread.detach();

    // start frame reading thread
    std::thread frame_reader([&]() {
        // Thread local storage for frame reading
        static thread_local cv::Mat local_frame(frame_height, frame_width, CV_8UC(frame_channels));

        bp::child ffmpeg_process(ffmpeg_exec, ffmpeg_args, bp::std_out > data_pipe, bp::std_err > error_pipe);

        while (!should_exit) {
            // Check if FFmpeg process is still running
            if (!ffmpeg_process.running()) {
                spdlog::error("FFmpeg process terminated unexpectedly");
                break;
            }

            // Read frame with timeout
            bool read_success = false;
            try {
                read_success = (bool)data_pipe.read(reinterpret_cast<char *>(local_frame.data), frame_size);

                if (read_success) {
                    std::lock_guard<std::mutex> lock(frame_mutex);
                    local_frame.copyTo(frame);
                    new_frame_available = true;
                    frame_cv.notify_one();
                    spdlog::info("read frame");
                }
            } catch (const std::exception &e) {
                spdlog::error("Error reading frame: {}", e.what());
                break;
            }

            if (!read_success) {
                spdlog::warn("Failed to read frame");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // Cleanup
        if (ffmpeg_process.running()) {
            ffmpeg_process.terminate();
        }
    });

    // Main thread handles display
    cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat display_frame;
        {
            std::unique_lock<std::mutex> lock(frame_mutex);
            frame_cv.wait(lock, [&] { return new_frame_available; });
            frame.copyTo(display_frame);
            new_frame_available = false;
        }

        cv::imshow("frame", display_frame);
        int key = cv::waitKey(1);
        if (key == 27) { // ESC key
            should_exit = true;
            break;
        }
    }

    // Cleanup and join threads
    if (frame_reader.joinable()) {
        frame_reader.join();
    }

    cv::destroyAllWindows();
    return 0;
}
