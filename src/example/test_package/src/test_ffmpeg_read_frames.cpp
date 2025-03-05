
#include <boost/process.hpp>
#include <boost/asio.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/thread/synchronized_value.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>
#include <string>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

namespace bp = boost::process;
namespace bi = boost::interprocess;
const std::string stderr_test_command = "while true; do echo \"current time is $(date +%H-%M-%S)\" >&2; sleep 1; done";

class FFmpegReaderByPipe
{
  private:
    boost::asio::io_context io_context_;
    boost::asio::streambuf logging_buffer_;
    bp::child ffmpeg_process_;
    bp::async_pipe pipe_out_async_;
    bp::async_pipe pipe_err_async_;

    // frame data
    int frame_width_ = 1920;
    int frame_height_ = 1080;
    int frame_channels_ = 3;
    cv::Mat frame_input_buf_;
    boost::synchronized_value<cv::Mat> frame_output_buf_;

  public:
    FFmpegReaderByPipe()
        : pipe_out_async_(io_context_), pipe_err_async_(io_context_)
    {
        frame_input_buf_ = cv::Mat(frame_height_, frame_width_, CV_8UC(frame_channels_));
        frame_output_buf_ = frame_input_buf_.clone();

        //! Construct ffmpeg command to read from /dev/video0
        std::vector<std::string> args = {
            "-framerate", "30",
            "-input_format", "mjpeg",
            "-f", "v4l2",
            "-fflags", "nobuffer", "-flags", "low_delay",
            "-discard", "nokey",
            "-i", "/dev/video0", // Input from webcam
            "-f", "rawvideo",
            "-video_size", "1920x1080",
            "-pix_fmt", "bgr24", // Set pixel format to BGR24
            "-",                 // output to stdout
        };

        //! Launch ffmpeg process with pipes for stdout and stderr
        // ffmpeg_process_ = bp::child(
        //     "/usr/bin/ffmpeg",
        //     args,
        //     bp::std_out > pipe_out_async_,
        //     bp::std_err > pipe_err_async_,
        //     io_context_);
        ffmpeg_process_ = bp::child(
            "/usr/bin/echo",
            std::vector<std::string>{"hello", "world"},
            (bp::std_out & bp::std_err) > "my_output.txt",
            io_context_);
    }

    //! Start async reading from stderr pipe
    void asyncReadStderr()
    {
        boost::asio::async_read_until(
            pipe_err_async_,
            logging_buffer_,
            "\n",
            [this](const boost::system::error_code &ec, std::size_t size) {
                if (!ec) {
                    //! Print the received log data
                    // spdlog::info(std::string(buffer_.data(), size));
                    std::istream is(&logging_buffer_);
                    std::string line;
                    std::getline(is, line);
                    std::cout << line << std::endl;

                    //! Continue reading
                    asyncReadStderr();
                }
            });
    }

    void asyncReadStdout()
    {
        boost::asio::async_read(
            pipe_out_async_,
            boost::asio::buffer(frame_input_buf_.data, frame_input_buf_.total() * frame_input_buf_.elemSize()),
            boost::asio::transfer_exactly(frame_input_buf_.total() * frame_input_buf_.elemSize()),
            [this](const boost::system::error_code &ec, std::size_t size) {
                if (!ec) {
                    auto buf = frame_output_buf_.try_to_synchronize();
                    if (buf) {
                        spdlog::info("read frame, copy to output buffer");
                        frame_input_buf_.copyTo(*buf);

                        // auto mean_pixel = cv::mean(*buf);
                        // spdlog::info("mean pixel: {},{},{}", mean_pixel[0], mean_pixel[1], mean_pixel[2]);
                    }

                    // async read next frame
                    asyncReadStdout();
                } else {
                    spdlog::error("Error reading frame: {}", ec.message());
                }
            });
    }

    bool readStderrOnce()
    {
        try {
            boost::asio::read_until(pipe_err_async_, logging_buffer_, "\n");
            std::istream is(&logging_buffer_);
            std::string line;
            std::getline(is, line);
            std::cout << line << std::endl;
            std::cout.flush();
            return true;
        } catch (const std::exception &e) {
            spdlog::error("Error reading stderr: {}", e.what());
            return false;
        }
    }

    bool readStdoutOnce()
    {
        auto buffer_size = frame_input_buf_.total() * frame_input_buf_.elemSize();
        try {
            auto num_bytes_read = boost::asio::read(pipe_out_async_,
                                                    boost::asio::buffer(frame_input_buf_.data, buffer_size));
            if (num_bytes_read == buffer_size) {
                // reading is ok, now copy to output buffer
                auto buf = frame_output_buf_.try_to_synchronize();
                if (buf) {
                    frame_input_buf_.copyTo(*buf);
                }
            }
            return true;
        } catch (const std::exception &e) {
            spdlog::error("Error reading stdout: {}", e.what());
            return false;
        }
    }

    //! Run the io_context to process async operations
    void run()
    {
        //! Start async reading from stderr pipe
        // asyncReadStderr();
        //! Start async reading from stdout pipe
        // asyncReadStdout();
        std::thread read_stderr_thread([this]() {
            while (readStderrOnce()) {
            }
        });

        // std::thread read_stdout_thread([this]() {
        //     while (readStdoutOnce()) {
        //     }
        // });

        //! Run the io_context to process async operations
        // std::thread io_thread([this]() {
        //     io_context_.run();
        // });

        // cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
        // auto frame_show_buf = frame_output_buf_->clone();
        // while (ffmpeg_process_.running()) {
        //     {
        //         auto frame_data = frame_output_buf_.synchronize();
        //         frame_data->copyTo(frame_show_buf);
        //     }
        //     // auto mean_pixel = cv::mean(frame_show_buf);
        //     // spdlog::info("imshow mean pixel: {},{},{}", mean_pixel[0], mean_pixel[1], mean_pixel[2]);
        //     cv::imshow("frame", frame_show_buf);
        //     cv::waitKey(1);
        // }
        read_stderr_thread.join();
        // read_stdout_thread.join();
        // io_thread.join();
    }

    //! Destructor to ensure cleanup
    ~FFmpegReaderByPipe()
    {
        if (ffmpeg_process_.running()) {
            ffmpeg_process_.terminate();
        }
    }
};

int main()
{
    try {
        FFmpegReaderByPipe reader;
        reader.run();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}