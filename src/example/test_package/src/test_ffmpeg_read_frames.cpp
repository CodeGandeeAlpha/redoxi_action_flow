
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

class FFmpegReaderByShm
{
  private:
    // ffmpeg process control
    boost::asio::io_context io_context_;
    std::vector<char> logging_buffer_;
    bp::child ffmpeg_process_;
    bp::async_pipe pipe_err_;

    // shared memory
    int frame_width_ = 1920;
    int frame_height_ = 1080;
    int frame_channels_ = 3;
    bi::shared_memory_object shm_obj_;
    bi::mapped_region shm_region_;
    cv::Mat frame_input_buf_;
    boost::synchronized_value<cv::Mat> frame_output_buf_;

  public:
    FFmpegReaderByShm()
        : pipe_err_(io_context_),
          shm_obj_(bi::create_only, "ffmpeg_shm", bi::read_write)
    {
        shm_obj_.truncate(frame_width_ * frame_height_ * frame_channels_);
        shm_region_ = bi::mapped_region(shm_obj_, bi::read_only);

        //! Construct ffmpeg command to read from /dev/video0
        std::vector<std::string> args = {
            "-framerate",
            "30",
            "-input_format",
            "mjpeg",
            "-f",
            "v4l2",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-i",
            "/dev/video0", // Input from webcam
            "-f",
            "rawvideo",
            "-video_size",
            "1920x1080",
            "-pix_fmt",
            "bgr24", // Set pixel format to BGR24
            "/dev/shm/ffmpeg_shm",
        };

        //! Launch ffmpeg process with pipes for stdout and stderr
        ffmpeg_process_ = bp::child(
            "/usr/bin/ffmpeg",
            args,
            bp::std_err > pipe_err_,
            io_context_);
    }

    //! Start async reading from stderr pipe
    void asyncReadStderr()
    {
        boost::asio::async_read(
            pipe_err_,
            boost::asio::buffer(logging_buffer_),
            [this](const boost::system::error_code &ec, std::size_t size) {
                if (!ec) {
                    //! Print the received log data
                    // spdlog::info(std::string(buffer_.data(), size));
                    std::cout.write(logging_buffer_.data(), size);
                    std::cout.flush();

                    //! Continue reading
                    asyncReadStderr();
                }
            });
    }

    void asyncReadShm()
    {
    }

    //! Run the io_context to process async operations
    void run()
    {
        //! Start async reading from stderr pipe
        asyncReadStderr();

        //! Start async reading from stdout pipe
        asyncReadShm();

        //! Run the io_context to process async operations
        std::thread io_thread([this]() {
            io_context_.run();
        });

        cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
        auto frame_show_buf = frame_output_buf_->clone();
        while (ffmpeg_process_.running()) {
            {
                auto frame_data = frame_output_buf_.synchronize();
                frame_data->copyTo(frame_show_buf);
            }
            auto mean_pixel = cv::mean(frame_show_buf);
            spdlog::info("imshow mean pixel: {},{},{}", mean_pixel[0], mean_pixel[1], mean_pixel[2]);
            cv::imshow("frame", frame_show_buf);
            cv::waitKey(1);
        }
        io_thread.join();
    }

    //! Destructor to ensure cleanup
    ~FFmpegReaderByShm()
    {
        if (ffmpeg_process_.running()) {
            ffmpeg_process_.terminate();
        }
        shm_obj_.remove("ffmpeg_shm");
    }
};

class FFmpegReaderByPipe
{
  private:
    boost::asio::io_context io_context_;
    std::vector<char> buffer_;
    bp::child ffmpeg_process_;
    bp::async_pipe pipe_out_;
    bp::async_pipe pipe_err_;

    // frame data
    int frame_width_ = 1920;
    int frame_height_ = 1080;
    int frame_channels_ = 3;
    cv::Mat frame_input_buf_;
    boost::synchronized_value<cv::Mat> frame_output_buf_;

  public:
    FFmpegReaderByPipe()
        : buffer_(20), pipe_out_(io_context_), pipe_err_(io_context_)
    {
        frame_input_buf_ = cv::Mat(frame_height_, frame_width_, CV_8UC(frame_channels_));
        frame_output_buf_ = frame_input_buf_.clone();

        //! Construct ffmpeg command to read from /dev/video0
        std::vector<std::string> args = {
            "-hwaccel", "cuvid", "-c:v", "mjpeg_cuvid",
            "-framerate", "30",
            "-input_format", "mjpeg",
            "-f", "v4l2",
            "-fflags", "nobuffer", "-flags", "low_delay",
            "-i", "/dev/video0", // Input from webcam
            "-f", "rawvideo",
            "-video_size", "1920x1080",
            "-pix_fmt", "bgr24", // Set pixel format to BGR24
            "-",                 // output to stdout
        };

        //! Launch ffmpeg process with pipes for stdout and stderr
        ffmpeg_process_ = bp::child(
            "/usr/bin/ffmpeg",
            args,
            bp::std_out > pipe_out_,
            bp::std_err > pipe_err_,
            io_context_);
    }

    //! Start async reading from stderr pipe
    void asyncReadStderr()
    {
        boost::asio::async_read(
            pipe_err_,
            boost::asio::buffer(buffer_),
            [this](const boost::system::error_code &ec, std::size_t size) {
                if (!ec) {
                    //! Print the received log data
                    // spdlog::info(std::string(buffer_.data(), size));
                    std::cout.write(buffer_.data(), size);
                    std::cout.flush();

                    //! Continue reading
                    asyncReadStderr();
                }
            });
    }

    void asyncReadStdout()
    {
        boost::asio::async_read(
            pipe_out_,
            boost::asio::buffer(frame_input_buf_.data, frame_input_buf_.total() * frame_input_buf_.elemSize()),
            boost::asio::transfer_exactly(frame_input_buf_.total() * frame_input_buf_.elemSize()),
            [this](const boost::system::error_code &ec, std::size_t size) {
                if (!ec) {
                    auto buf = frame_output_buf_.try_to_synchronize();
                    if (buf) {
                        spdlog::info("read frame, copy to output buffer");
                        frame_input_buf_.copyTo(*buf);

                        auto mean_pixel = cv::mean(*buf);
                        spdlog::info("mean pixel: {},{},{}", mean_pixel[0], mean_pixel[1], mean_pixel[2]);
                    }

                    // async read next frame
                    asyncReadStdout();
                } else {
                    spdlog::error("Error reading frame: {}", ec.message());
                }
            });
    }

    //! Run the io_context to process async operations
    void run()
    {
        //! Start async reading from stderr pipe
        asyncReadStderr();

        //! Start async reading from stdout pipe
        asyncReadStdout();

        //! Run the io_context to process async operations
        std::thread io_thread([this]() {
            io_context_.run();
        });

        cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
        auto frame_show_buf = frame_output_buf_->clone();
        while (ffmpeg_process_.running()) {
            {
                auto frame_data = frame_output_buf_.synchronize();
                frame_data->copyTo(frame_show_buf);
            }
            auto mean_pixel = cv::mean(frame_show_buf);
            spdlog::info("imshow mean pixel: {},{},{}", mean_pixel[0], mean_pixel[1], mean_pixel[2]);
            cv::imshow("frame", frame_show_buf);
            cv::waitKey(1);
        }
        io_thread.join();
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