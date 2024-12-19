#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <opencv2/opencv.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace redoxi_works::image_utils
{
/**
 * @brief Compute the size to resize to, keeping the aspect ratio, so that the resized image fits into the preferred size.
 * @param original_size The size of the original image.
 * @param preferred_size The preferred size of the resized image.
 * @return The size to resize to.
 */
cv::Size compute_resize_to_fit_and_keep_aspect_ratio(const cv::Size &original_size, const cv::Size &preferred_size);

/**
 * @brief Options for drawing detections on an image.
 */
struct DrawDetectionsOptions {
    enum class ColorizationMode {
        None,             //!< if color is defined in the detection object, use it, otherwise use the default color
        ClassId,          //!< Colorize by class id
        SemanticIdentity, //!< Colorize by semantic identity
        Random            //!< Random colorization
    };

    ColorizationMode colorization_mode = ColorizationMode::None;
    cv::Scalar default_box_color = cv::Scalar(0, 0, 255);
    bool draw_bboxes = true;
    int box_thickness = 2;
    bool draw_keypoints = true;
    int keypoint_radius = 2;
};

/**
 * @brief Draw detections on an image.
 */
void draw_detections(cv::Mat *output,
                     const std::vector<redoxi_public_msgs::msg::Detection> &detections,
                     const DrawDetectionsOptions &options = DrawDetectionsOptions());

// map cv type to ros encoding
template <int T>
constexpr const char *cv_type_to_ros_encoding()
{
    switch (T) {
        case CV_8UC3:
            return sensor_msgs::image_encodings::RGB8;
        case CV_8UC1:
            return sensor_msgs::image_encodings::MONO8;
        case CV_16UC3:
            return sensor_msgs::image_encodings::RGB16;
        case CV_16UC1:
            return sensor_msgs::image_encodings::MONO16;
        default:
            return "";
    }
}

//! Get the default image encoding for a given image, based on its type and number of channels
inline std::string get_default_image_encoding(const cv::Mat &image)
{
    auto cvtype = image.type();
    switch (cvtype) {
        case CV_8UC3:
            return cv_type_to_ros_encoding<CV_8UC3>();
        case CV_16UC3:
            return cv_type_to_ros_encoding<CV_16UC3>();
        case CV_8UC1:
            return cv_type_to_ros_encoding<CV_8UC1>();
        case CV_16UC1:
            return cv_type_to_ros_encoding<CV_16UC1>();
        default:
            return "";
    }
}

//! convert cv type to tensor dtype
//! @note for unsupported types, return empty string
template <int CvType>
inline constexpr const char *cv_type_to_string()
{
    //! Get depth of cv type (e.g. CV_8U for CV_8UC3)
    auto depth = CV_MAT_DEPTH(CvType);
    switch (depth) {
        case CV_8U:
            return "uint8";
        case CV_16U:
            return "uint16";
        case CV_8S:
            return "int8";
        case CV_16S:
            return "int16";
        case CV_32S:
            return "int32";
        case CV_32F:
            return "float32";
        case CV_64F:
            return "float64";
        default:
            return "";
    }
}

//! convert cv type to tensor dtype
//! @note for unsupported types, return empty string
inline const char *cv_type_to_string(int cvtype)
{
    //! Get depth of cv type (e.g. CV_8U for CV_8UC3)
    auto depth = CV_MAT_DEPTH(cvtype);
    switch (depth) {
        case CV_8U:
            return cv_type_to_string<CV_8U>();
        case CV_16U:
            return cv_type_to_string<CV_16U>();
        case CV_8S:
            return cv_type_to_string<CV_8S>();
        case CV_16S:
            return cv_type_to_string<CV_16S>();
        case CV_32S:
            return cv_type_to_string<CV_32S>();
        case CV_32F:
            return cv_type_to_string<CV_32F>();
        case CV_64F:
            return cv_type_to_string<CV_64F>();
        default:
            return "";
    }
}

} // namespace redoxi_works::image_utils
