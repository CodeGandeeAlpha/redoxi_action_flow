#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <algorithm>
#include <numeric>

namespace redoxi_works::image_utils
{
cv::Size compute_resize_to_fit_and_keep_aspect_ratio(const cv::Size &original_size, const cv::Size &preferred_size)
{
    auto resize_ratio = std::min(preferred_size.width / (float)original_size.width,
                                 preferred_size.height / (float)original_size.height);

    // no clamping, using rounding
    cv::Size output(
        static_cast<int>(std::round(original_size.width * resize_ratio)),
        static_cast<int>(std::round(original_size.height * resize_ratio)));

    // no 0 size
    output.width = std::max(1, output.width);
    output.height = std::max(1, output.height);

    return output;
}
} // namespace redoxi_works::image_utils
