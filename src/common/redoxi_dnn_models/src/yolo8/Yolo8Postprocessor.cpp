#include <redoxi_dnn_models/yolo8/Yolo8Postprocessor.hpp>
#include <opencv2/dnn/dnn.hpp>

namespace redoxi_works::inference::yolo8
{

void Postprocessor::init(const PostprocessorConfig &config)
{
    m_config = std::make_shared<PostprocessorConfig>(config);
}

void Postprocessor::postprocess(
    SingleImageOutput::List *output_result,
    const float *model_output_batch_values_numboxes,
    const std::array<int64_t, 3> &model_output_shape,
    const ImagePreprocessInfo::List &preprocess_info) const
{
    // nothing to do if output_result is not provided
    if (!output_result) {
        return;
    }

    // get dimensions
    int64_t batch_size = model_output_shape[0];
    int64_t num_values = model_output_shape[1];
    int64_t num_boxes = model_output_shape[2];

    // prepare output list
    output_result->resize(batch_size);

    // process each image in batch
    for (int64_t i = 0; i < batch_size; i++) {
        // get pointer to data for this image
        const float *image_data = model_output_batch_values_numboxes + i * num_values * num_boxes;

        // get preprocess info for this image
        const auto &pinfo = preprocess_info[i];

        // process single image
        std::array<int64_t, 2> single_output_shape = {num_values, num_boxes};
        postprocess(&(*output_result)[i], image_data, single_output_shape, pinfo);
    }
}


} // namespace redoxi_works::inference::yolo8
