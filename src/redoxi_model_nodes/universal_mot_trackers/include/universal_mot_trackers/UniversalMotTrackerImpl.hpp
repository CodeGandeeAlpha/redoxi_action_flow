#pragma once

#include <universal_mot_trackers/UniversalMotTrackerNode.hpp>
#include <RedoxiTrack/RedoxiTrack.h>
#include <variant>

namespace redoxi_works::model_nodes::universal_mot_trackers
{

struct BaseTrackerImpl {
    virtual ~BaseTrackerImpl() = default;
    BaseTrackerImpl()
    {
        m_tracker_event_handler = std::make_shared<RedoxiTrack::ExternalTrackingEventHandler>();
    }

    using Ptr = std::shared_ptr<BaseTrackerImpl>;
    using ConstPtr = std::shared_ptr<const BaseTrackerImpl>;
    std::shared_ptr<RedoxiTrack::TrackerBase> m_tracker;
    std::shared_ptr<RedoxiTrack::TrackerParam> m_tracker_param;
    RedoxiTrack::ExternalTrackingEventHandler::Ptr m_tracker_event_handler;
};

struct DeepsortImpl : public BaseTrackerImpl {
    using Ptr = std::shared_ptr<DeepsortImpl>;
    using ConstPtr = std::shared_ptr<const DeepsortImpl>;

    DeepsortImpl()
    {
        m_tracker = std::make_shared<RedoxiTrack::DeepSortTracker>();
        m_tracker_param = std::make_shared<RedoxiTrack::DeepSortTrackerParam>();
    }
};

struct BotsortImpl : public BaseTrackerImpl {
    using Ptr = std::shared_ptr<BotsortImpl>;
    using ConstPtr = std::shared_ptr<const BotsortImpl>;

    BotsortImpl()
    {
        m_tracker = std::make_shared<RedoxiTrack::BotsortTracker>();
        m_tracker_param = std::make_shared<RedoxiTrack::BotsortTrackerParam>();
    }
};

struct UniversalMotTrackerNode::Impl {
    BaseTrackerImpl::Ptr m_tracker_impl;
};
} // namespace redoxi_works::model_nodes::universal_mot_trackers
