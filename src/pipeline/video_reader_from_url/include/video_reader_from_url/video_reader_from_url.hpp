#pragma once

#include <redoxi_video_reader/VideoSourceFromUrl.hpp>

#include "video_reader_from_url/visibility_control.h"

namespace redoxi_works
{

class VideoReaderFromUrl : public video_readers::VideoSourceFromUrl
{
  public:
    using VideoSourceFromUrl::VideoSourceFromUrl;

  protected:
    virtual int _on_before_request_enqueue(DeliveryRequest_t &request, DeliveryPolicy_t &enqueue_policy) override;
};

} // namespace redoxi_works