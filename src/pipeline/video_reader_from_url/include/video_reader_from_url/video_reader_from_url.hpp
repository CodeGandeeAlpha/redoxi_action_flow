#pragma once

#include <redoxi_video_reader/VideoSourceFromUrl.hpp>

#include "video_reader_from_url/visibility_control.h"

namespace redoxi_works
{

class VideoReaderFromUrl : public video_readers::VideoSourceFromUrl
{
  public:
    using VideoSourceFromUrl::VideoSourceFromUrl;

    struct InitConfig : public VideoSourceFromUrl::InitConfig_t {
        // read video from this file
        std::string crop_cfg_path;

        JS_OBJECT_WITH_SUPER(
            JS_SUPER(VideoSourceFromUrl::InitConfig_t),
            JS_MEMBER(crop_cfg_path));
    };

    using InitConfig_t = VideoReaderFromUrl::InitConfig;
    using RuntimeConfig_t = VideoSourceFromUrl::RuntimeConfig_t;

  protected:
    int _open() override;
    virtual int _on_before_request_enqueue(DeliveryRequest_t &request, DeliveryPolicy_t &enqueue_policy) override;

    std::string m_crop_cfg_json_str;
};

} // namespace redoxi_works
