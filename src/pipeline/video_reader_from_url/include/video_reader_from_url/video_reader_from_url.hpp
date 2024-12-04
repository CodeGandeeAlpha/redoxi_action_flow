#pragma once

#include <redoxi_video_reader/VideoSourceFromUrl.hpp>

#include "video_reader_from_url/visibility_control.h"

namespace redoxi_works
{

class VideoReaderFromUrl : public VideoSourceFromUrl
{
  public:
    using VideoSourceFromUrl::VideoSourceFromUrl;

  protected:
    virtual ReadFrameResult _read_frame(SourceData_t &source_data,
                                        std::atomic<int64_t> &frame_number) override;
};

} // namespace redoxi_works