#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/Yolo8PoseTypes.hpp>
#include <redoxi_inference/redoxi_inference.hpp>
#include <opencv2/opencv.hpp>
#include <array>

namespace redoxi_works::inference
{

class Yolo8Pose : public RedoxiModelInference
{
  public:
    // keypoint in the image
    struct Keypoint {
        std::array<float, 2> xy = {0, 0};
        float score = 0;
    };

    // detected object in the image
    struct DetectedObject {
        int64_t class_id = 0;
        std::array<float, 4> xywh = {0, 0, 0, 0};
        float score = 0;
        std::vector<Keypoint> keypoints;
    };

    // output of the model for a single image
    struct SingleImageOutput {
        std::vector<DetectedObject> objects;
    };

    // // information about how the preprocess transforms the original input image to the model input image
    // // it selects a region of the source image (full size original image), and then transform that region to the model input image
    // struct ImagePreprocessInfo {
    //     using List = std::vector<ImagePreprocessInfo>;

    //     // size of the original input image
    //     cv::Size source_image_size;

    //     // a box in the original input image, enclosing all the content of the model input image
    //     cv::Rect roi_in_source_image;

    //     // size of the model input image
    //     cv::Size model_input_image_size;

    //     // a box in the model input image, enclosing all the content of the original input image
    //     cv::Rect roi_in_model_input_image;
    // };

    using InitConfig_t = Yolo8PoseConfig;

  public:
    Yolo8Pose();
    // from RedoxiModelInference
    virtual KeyValueStore::Ptr create_init_params() override;
    virtual InferenceInOutData::Ptr create_inference_inout_data() override;
    virtual ModelPortInfo::ConstPtrMap get_input_port_infos() const override;
    virtual ModelPortInfo::ConstPtrMap get_output_port_infos() const override;
    virtual int open(KeyValueStore::Ptr params) override;
    virtual bool is_open() const override;
    virtual int close() override;
    virtual KeyValueStore::ConstPtr get_model_metadata() const override;
    virtual KeyValueStore::ConstPtr get_inference_metadata() const override;
    virtual int do_inference(InferenceInOutData::Ptr inout_data) override;

  public:
    // Yolo8Pose specific

    // process the images, and set the data to the model input
    // the images must be of the same size, in RGB format
    int set_input_images_v1(InferenceInOutData::Ptr model_inout_data,
                            const std::vector<cv::Mat> &rgb_images);

    // process the images, and set the data to the model input
    // the images must be of the same size, in RGB format
    int set_input_images(InferenceInOutData::Ptr model_inout_data,
                         const std::vector<cv::Mat> &rgb_images);

    // postprocess the model output, and get the detections
    std::vector<SingleImageOutput> get_output_detections(
        InferenceInOutData::Ptr model_inout_data,
        double confidence_thres = 0.5) const;

    // get the shape of the model input in NCHW format
    std::array<int64_t, 4> get_model_input_shape_nchw() const;
    std::string get_model_input_dtype() const;

    // get the shape of the model output in NCHW format
    std::array<int64_t, 3> get_model_output_shape_nchw() const;
    std::string get_model_output_dtype() const;

  protected:
    struct Impl;
    std::shared_ptr<Impl> m_impl;

    // the actual model
    std::shared_ptr<RedoxiModelInference> m_model;

    // information about the model input and output
    ModelPortInfo::ConstPtr m_model_input_info;
    ModelPortInfo::ConstPtr m_model_output_info;

    std::shared_ptr<InitConfig_t> m_init_params;
};
} // namespace redoxi_works::inference
