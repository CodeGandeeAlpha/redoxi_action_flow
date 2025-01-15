#include "video_reader_from_url/video_reader_from_url.hpp"
#include <fstream>

namespace redoxi_works
{

int VideoReaderFromUrl::_open()
{
    //! 首先调用父类的 _open 实现（如果需要的话）
    int result = video_readers::VideoSourceFromUrl::_open();
    if (result != 0) {
        return result;
    }

    //! 获取初始化配置
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    RDX_INFO_DEV(this, __func__, "init_config->crop_cfg_path: {}", init_config->crop_cfg_path);

    //! 读取裁剪配置
    if (!init_config->crop_cfg_path.empty()) {
        std::ifstream crop_cfg_file(init_config->crop_cfg_path);
        if (!crop_cfg_file.is_open()) {
            RDX_LOG_ERROR(this, __func__, "打开crop配置文件失败: {}", init_config->crop_cfg_path);
            return -1;
        }

        //! 读取视频长宽
        int video_width = m_video_capture->get(cv::CAP_PROP_FRAME_WIDTH);
        int video_height = m_video_capture->get(cv::CAP_PROP_FRAME_HEIGHT);
        RDX_INFO_DEV(this, __func__, "video_width: {}, video_height: {}", video_width, video_height);

        std::string crop_cfg_json_str((std::istreambuf_iterator<char>(crop_cfg_file)), std::istreambuf_iterator<char>());
        //! 根据视频长宽与runtime_config中的output_image_size，生成crop配置
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
        int output_image_width = runtime_config->output_image_size.width;
        int output_image_height = runtime_config->output_image_size.height;
        RDX_INFO_DEV(this, __func__, "output_image_width: {}, output_image_height: {}", output_image_width, output_image_height);

        auto crop_json_data = nlohmann::json::parse(crop_cfg_json_str);
        double scale_x = 0;
        double scale_y = 0;
        if (output_image_width > 0 && output_image_height > 0) {
            scale_x = static_cast<double>(output_image_width) / video_width;
            scale_y = static_cast<double>(output_image_height) / video_height;
        } else if (output_image_width > 0) {
            scale_x = static_cast<double>(output_image_width) / video_width;
            scale_y = scale_x;
        } else if (output_image_height > 0) {
            scale_y = static_cast<double>(output_image_height) / video_height;
            scale_x = scale_y;
        }
        crop_json_data["top_left"][0] = static_cast<int>(crop_json_data["top_left"][0].get<int>() * scale_x);
        crop_json_data["top_left"][1] = static_cast<int>(crop_json_data["top_left"][1].get<int>() * scale_y);
        crop_json_data["bottom_right"][0] = static_cast<int>(crop_json_data["bottom_right"][0].get<int>() * scale_x);
        crop_json_data["bottom_right"][1] = static_cast<int>(crop_json_data["bottom_right"][1].get<int>() * scale_y);
        crop_json_data["w"] = static_cast<int>(crop_json_data["w"].get<int>() * scale_x);
        crop_json_data["h"] = static_cast<int>(crop_json_data["h"].get<int>() * scale_y);
        m_crop_cfg_json_str = crop_json_data.dump();
        RDX_INFO_DEV(this, __func__, "根据视频画面长宽与output_image_size，生成crop配置: {}", m_crop_cfg_json_str);
    } else {
        RDX_INFO_DEV(this, __func__, "{}", "crop配置路径为空，跳过读取crop配置");
    }

    return 0;
}

int VideoReaderFromUrl::_on_before_request_enqueue(DeliveryRequest_t &request, DeliveryPolicy_t &enqueue_policy)
{
    // auto request_policy = request.get_delivery_policy();
    // if (!request_policy) {
    //     DeliveryPolicy_t policy;
    //     policy.set_precondition(DeliveryPrecondition::NoPrecondition);
    //     policy.set_drop_strategy(DropStrategy::NoDrop);
    //     request.set_delivery_policy(policy);
    // } else {
    //     request_policy->set_precondition(DeliveryPrecondition::NoPrecondition);
    //     request_policy->set_drop_strategy(DropStrategy::NoDrop);
    // }
    // request.set_control_signal_code(ControlSignalCode::Flush);

    auto &source_data = request.get_source_data();
    //! 设置裁剪配置到帧元数据的any_data字段
    if (!m_crop_cfg_json_str.empty()) {
        source_data.get_primary_frame().get_metadata().any_data.string_data = m_crop_cfg_json_str;
    }
    RDX_INFO_DEV(this, __func__, "frame num: {}", source_data.get_primary_frame().get_metadata().frame_num);
    (void)enqueue_policy;
    return 0;
}
} // namespace redoxi_works
