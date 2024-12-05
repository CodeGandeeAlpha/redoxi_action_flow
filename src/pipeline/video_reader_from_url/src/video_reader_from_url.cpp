#include "video_reader_from_url/video_reader_from_url.hpp"

namespace redoxi_works
{

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

    auto source_data = request.get_source_data();
    RDX_INFO_DEV(this, __func__, "frame num: {}", source_data.get_frame_metadata().frame_num);
    (void)enqueue_policy;
    return 0;
}
} // namespace redoxi_works
