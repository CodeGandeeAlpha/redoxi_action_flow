#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <opencv2/opencv.hpp>

namespace redoxi_works::inference::detection::types
{
// keypoint in the image
struct Keypoint {
    std::array<float, 2> xy = {0, 0};
    float score = 0;
};

// detected object in the image
struct DetectedObject {
    int64_t class_id = 0;
    std::string class_name;
    std::array<float, 4> xywh = {0, 0, 0, 0};
    float score = 0;
    std::vector<Keypoint> keypoints;
};

// output of the model for a single image
struct SingleImageOutput {
    using List = std::vector<SingleImageOutput>;
    std::vector<DetectedObject> objects;
};

// information about how the preprocess transforms the original input image to the model input image
// it selects a region of the source image (full size original image), and then transform that region to the model input image
struct ImagePreprocessInfo {
    using List = std::vector<ImagePreprocessInfo>;

    // size of the original input image
    cv::Size source_image_size;

    // a box in the original input image, enclosing all the content of the model input image
    cv::Rect roi_in_source_image;

    // size of the model input image
    cv::Size model_input_image_size;

    // a box in the model input image, enclosing all the content of the original input image
    cv::Rect roi_in_model_input_image;
};
} // namespace redoxi_works::inference::detection::types