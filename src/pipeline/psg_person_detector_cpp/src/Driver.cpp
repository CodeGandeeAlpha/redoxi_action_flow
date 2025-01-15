#include <psg_person_detector_cpp/Driver.hpp>

namespace redoxi_works
{

int PSGPersonDetectorCppDriver::_on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                                          OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                                          std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                                          const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                          const CalleeTypes::Downstream_t &downstream)
{
    (void)downstream;
    (void)output_enqueue_policy;

    OutputTypes::OutputSourceData_t output_pipeline_source_data;

    // 设置auxiliary_data的类型，用于可视化
    output_pipeline_source_data.auxiliary_data = std::string("detection");

    psg_private_msgs::msg::PsgDocument document;
    auto &primary_frame = callee_request.get_source_data().get_primary_frame();

    // 如果crop_cfg_data不为空，则记录下左上角坐标
    int crop_x = 0;
    int crop_y = 0;
    if (!primary_frame.get_metadata().any_data.string_data.empty()) {
        // 将原始的frame_bundle设置到document
        document.frame_bundle = std::any_cast<redoxi_public_msgs::msg::MultiDeviceFrame>(callee_request.get_source_data().auxiliary_data);
        auto js_crop_data = nlohmann::json::parse(document.frame_bundle.primary_frame.metadata.any_data.string_data);
        crop_x = js_crop_data["top_left"][0].get<int>();
        crop_y = js_crop_data["top_left"][1].get<int>();
        RDX_INFO_DEV(this, __func__, false, "crop_x: {}, crop_y: {}", crop_x, crop_y);
    } else {
        RDX_INFO_DEV(this, __func__, false, "{}", "crop_cfg_data为空");
        primary_frame.to_frame_mediator().to_frame_msg(document.frame_bundle.primary_frame);
    }

    auto is_with_head_detection = !callee_result->detections.empty() && callee_result->detections[0].keypoints.keypoints_2.size() == 19;

    // 根据detections中的keypoints的头点，构建头的bbox
    if (!is_with_head_detection) {
        for (const auto &detection : callee_result->detections) {
            auto head_keypoint = detection.keypoints.keypoints_2[0];
            auto body_width = detection.bbox.width;
            auto body_height = detection.bbox.height;
            // 恢复到原始的坐标
            head_keypoint.x += crop_x;
            head_keypoint.y += crop_y;

            auto head_bbox = cv::Rect(head_keypoint.x - body_width / 3 / 2,
                                      head_keypoint.y - body_height / 6 / 2,
                                      body_width / 3,
                                      body_height / 6);
            redoxi_public_msgs::msg::Detection head_detection;
            head_detection.bbox.x = head_bbox.x;
            head_detection.bbox.y = head_bbox.y;
            head_detection.bbox.width = head_bbox.width;
            head_detection.bbox.height = head_bbox.height;
            head_detection.category = 1;
            head_detection.confidence = 1.0;
            head_detection.frame_metadata = detection.frame_metadata; // 不加这个会导致event中start_time和end_time为-1
            // head_detection.is_detected_by_camera = true;
            document.detections.push_back(head_detection);

            // 新建人体的detection
            redoxi_public_msgs::msg::Detection body_detection;
            body_detection.bbox = detection.bbox;
            // 恢复到原始的坐标
            body_detection.bbox.x += crop_x;
            body_detection.bbox.y += crop_y;
            body_detection.category = 0;
            body_detection.confidence = detection.confidence;
            for (size_t i = 0; i < detection.keypoints.keypoints_2.size(); ++i) {
                const auto &kp = detection.keypoints.keypoints_2[i];
                geometry_msgs::msg::Point point;
                point.x = kp.x + crop_x;
                point.y = kp.y + crop_y;
                point.z = 0;
                body_detection.keypoints.keypoints_2.push_back(point);
                body_detection.keypoints.confidence.push_back(detection.keypoints.confidence[i]);
                body_detection.keypoints.semantic_type.push_back(i);
            }
            body_detection.frame_metadata = detection.frame_metadata; // 不加这个会导致event中start_time和end_time为-1
            document.detections.push_back(body_detection);

            // 构建person
            psg_private_msgs::msg::Person person;
            person.true_head = head_detection;
            person.true_body = body_detection;
            person.frame_metadata = document.frame_bundle.primary_frame.metadata;
            person.x_uid = to_ros_uuid_msg(boost::uuids::random_generator()()); // 不加这个会导致tracker无法匹配
            document.persons.push_back(person);
        }
    } else {
        // 如果包含头部, 则将前两个关键点作为头部, 剩下17个关键点作为身体
        for (const auto &detection : callee_result->detections) {
            auto head_lt_keypoint = detection.keypoints.keypoints_2[0];
            auto head_rb_keypoint = detection.keypoints.keypoints_2[1];
            auto head_bbox = cv::Rect(head_lt_keypoint.x, head_lt_keypoint.y, head_rb_keypoint.x - head_lt_keypoint.x, head_rb_keypoint.y - head_lt_keypoint.y);
            redoxi_public_msgs::msg::Detection head_detection;
            head_detection.bbox.x = head_bbox.x + crop_x;
            head_detection.bbox.y = head_bbox.y + crop_y;
            head_detection.bbox.width = head_bbox.width;
            head_detection.bbox.height = head_bbox.height;
            head_detection.category = 1;
            head_detection.confidence = (detection.keypoints.confidence[0] + detection.keypoints.confidence[1]) / 2;
            // head_detection.is_detected_by_camera = true;
            document.detections.push_back(head_detection);

            // 新建人体的detection
            redoxi_public_msgs::msg::Detection body_detection;
            body_detection.bbox = detection.bbox;
            // 恢复到原始的坐标
            body_detection.bbox.x += crop_x;
            body_detection.bbox.y += crop_y;
            body_detection.category = 0;
            body_detection.confidence = detection.confidence;
            for (size_t i = 2; i < detection.keypoints.keypoints_2.size(); ++i) {
                const auto &kp = detection.keypoints.keypoints_2[i];
                geometry_msgs::msg::Point point;
                point.x = kp.x + crop_x;
                point.y = kp.y + crop_y;
                body_detection.keypoints.keypoints_2.push_back(point);
                body_detection.keypoints.confidence.push_back(detection.keypoints.confidence[i]);
                body_detection.keypoints.semantic_type.push_back(i - 2);
            }
            body_detection.frame_metadata = detection.frame_metadata; // 不加这个会导致event中start_time和end_time为-1
            document.detections.push_back(body_detection);

            // 构建person
            psg_private_msgs::msg::Person person;
            person.true_head = head_detection;
            person.true_body = body_detection;
            person.frame_metadata = document.frame_bundle.primary_frame.metadata;
            person.x_uid = to_ros_uuid_msg(boost::uuids::random_generator()()); // 不加这个会导致tracker无法匹配
            document.persons.push_back(person);
        }
    }


    output_pipeline_source_data.set_document(document);
    output_request->set_source_data(output_pipeline_source_data);

    const auto signal_code = callee_request.get_control_signal_code();
    output_request->set_control_signal_code(signal_code);
    return 0;
}

int PSGPersonDetectorCppDriver::_on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                                          std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                                          InputRequestHandler_t::InputActionResult_t *output_result,
                                                          std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                                          InputRequestHandler_t::ResourceToken_t &resource_token)
{
    (void)resource_token;
    (void)output_result;
    (void)output_enqueue_policy;


    // from input source data to output source data
    auto msg_uuid = InputTypes::ActionDataTrait_t::get_uuid(*source_data->get_goal());
    auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Creating request from source data", msg_uuid_str);
    CalleeTypes::RequestOutputSourceData_t output_source_data;

    auto &original_frame_bundle = source_data->get_goal()->document.frame_bundle;
    output_source_data.auxiliary_data = original_frame_bundle;
    CalleeTypes::RequestOutputSourceData_t::FrameData_t frame_data;
    frame_data.from_frame_msg(original_frame_bundle.primary_frame);

    // 将crop后的frame_bundle设置到output_source_data
    if (!original_frame_bundle.primary_frame.metadata.any_data.string_data.empty()) {
        auto js_crop_data = nlohmann::json::parse(original_frame_bundle.primary_frame.metadata.any_data.string_data.data());
        auto crop_rect = cv::Rect(js_crop_data["top_left"][0].get<int>(), js_crop_data["top_left"][1].get<int>(), js_crop_data["w"].get<int>(), js_crop_data["h"].get<int>());
        auto img = frame_data.to_frame_mediator().to_cv_image_copy();
        RDX_INFO_DEV(this, __func__, false, "img.shape: w{} h{}", img.size().width, img.size().height);
        cv::Mat cropped_img;
        if (!img.empty()) {
            cropped_img = img(crop_rect);
        }
        auto metadata = frame_data.get_metadata();
        metadata.width = cropped_img.size().width;
        metadata.height = cropped_img.size().height;
        frame_data.from_raw_data({.image = cropped_img, .metadata = metadata});
        RDX_INFO_DEV(this, __func__, false, "after from_raw_data, img_width: {}, img_height: {}", frame_data.get_metadata().width, frame_data.get_metadata().height);
    }

    output_source_data.set_primary_frame(frame_data);

    auto goal_handle = source_data->get_goal_handle_future().get();
    auto control_signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    RDX_INFO_DEV(this, __func__, true,
                 "on_process_input_data()中frame num: {}, control signal code: {}",
                 source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num, int(control_signal_code));


    // create delivery request
    output_request->set_source_data(output_source_data);
    const auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    output_request->set_control_signal_code(signal_code);


    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Set control signal code to {}",
                 msg_uuid_str, control_signal_code_to_string(signal_code));

    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Done, created request from source data", msg_uuid_str);

    return 0;
}

} // namespace redoxi_works