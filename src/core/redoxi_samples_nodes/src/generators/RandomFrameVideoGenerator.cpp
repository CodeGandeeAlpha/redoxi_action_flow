#include <redoxi_samples_nodes/_pch.hpp>

#include <redoxi_samples_nodes/generators/RandomFrameVideoGenerator.hpp>
#include <redoxi_samples_lib/random_image.hpp>
#include <redoxi_common_cpp/image_proc/ImageStamper.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace redoxi_works
{
RandomFrameVideoGenerator::RandomFrameVideoGenerator(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase(name, options)
{
}

int RandomFrameVideoGenerator::_update_runtime_config(std::shared_ptr<RedoxiVideoReaderBase::BaseRuntimeConfig_t> config)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);

    // ensure the output image size is valid, otherwise uses default size
    if (runtime_config->output_image_size.width <= 0 || runtime_config->output_image_size.height <= 0) {
        RCLCPP_WARN(this->get_logger(),
                    "[%s][update_runtime_config()] Invalid output_image_size (%d, %d). Using default size.",
                    this->get_name(), runtime_config->output_image_size.width, runtime_config->output_image_size.height);
        runtime_config->output_image_size = RuntimeConfig_t::DEFAULT_FRAME_SIZE;
    }

    //! Call the base class implementation
    int ret = RedoxiVideoReaderBase::_update_runtime_config(config);
    if (ret != 0) {
        return ret;
    }
    return 0;
}

RandomFrameVideoGenerator::ReadFrameResult
    RandomFrameVideoGenerator::_read_frame(SourceData_t &data, std::atomic<int64_t> &frame_number)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto frame_size = runtime_config->output_image_size;
    if (frame_size.empty()) {
        RDX_RAISE_ERROR("[{}][_read_frame()] output_image_size is not set", this->get_name());
    }

    //! Generate a random frame with the UUID text
    cv::Mat random_frame;
    auto uuid = data.get_uuid();
    auto frame_text = fmt::format("{}\nFrame Number: {}", boost::uuids::to_string(uuid), frame_number.load());
    random_image_with_text(random_frame, frame_size, frame_text);

    //! Convert the frame to the desired encoding and set it in the source data
    image_utils::FrameMediator fm(random_frame, runtime_config->output_image_encoding);
    data.set_uuid(uuid);
    data.get_primary_frame().from_raw_data({.image = fm.to_cv_image_shared(), .metadata = fm.get_metadata()});

    // must do it this way to ensure thread safety
    int64_t current_frame_number = _increment_frame_number_by(frame_number, 1);
    data.get_primary_frame().get_metadata().frame_num = current_frame_number;

    return ReadFrameResult::OK;
}

} // namespace redoxi_works
