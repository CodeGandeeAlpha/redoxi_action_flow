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

        std::string crop_cfg_json_str((std::istreambuf_iterator<char>(crop_cfg_file)), std::istreambuf_iterator<char>());
        m_crop_cfg_json_str = crop_cfg_json_str;
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
