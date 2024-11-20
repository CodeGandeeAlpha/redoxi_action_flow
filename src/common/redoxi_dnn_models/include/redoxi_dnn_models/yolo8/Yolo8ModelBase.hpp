#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_inference/redoxi_inference.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelTypes.hpp>
#include <opencv2/opencv.hpp>

namespace redoxi_works::inference::yolo8
{
class Yolo8ModelBase : public RedoxiModelInference
{
  public:
    // output types
    using Keypoint = Keypoint;
    using DetectedObject = DetectedObject;
    using SingleImageOutput = SingleImageOutput;
    using InitConfig_t = Yolo8ModelConfig;
    using OutputConfig_t = PostprocessorConfig;

  public:
    Yolo8ModelBase();
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
    // image format follows the ros2 sensor_msgs::image_encodings convention
    // can be "rgb8" or "bgr8" or "mono8", all images must be of the same format
    virtual int set_input_images(InferenceInOutData::Ptr model_inout_data,
                                 const std::vector<cv::Mat> &images,
                                 const std::string &image_format);

    // postprocess the model output, and get the detections
    virtual std::vector<SingleImageOutput> get_output_detections(
        InferenceInOutData::Ptr model_inout_data,
        const OutputConfig_t &config) const = 0;

    // get the shape of the model input in NCHW format
    virtual std::array<int64_t, 4> get_model_input_shape_nchw() const;
    virtual std::string get_model_input_dtype() const;

    // get the shape of the model output in (batch_size, num_values, num_boxes)
    virtual std::array<int64_t, 3> get_model_output_shape() const;
    virtual std::string get_model_output_dtype() const;

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

} // namespace redoxi_works::inference::yolo8
