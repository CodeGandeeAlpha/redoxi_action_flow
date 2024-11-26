#include <redoxi_dnn_models/message_conversion.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace redoxi_works::inference::conversion
{
void to_ros_msg(redoxi_public_msgs::msg::Detection *output_msg,
                const det_types::DetectedObject &object)
{
    if (!output_msg) {
        return;
    }

    output_msg->category = object.class_id;
    output_msg->category_name = object.class_name;
    output_msg->confidence = object.score;
    output_msg->bbox.x = object.xywh[0];
    output_msg->bbox.y = object.xywh[1];
    output_msg->bbox.width = object.xywh[2];
    output_msg->bbox.height = object.xywh[3];

    // Convert keypoints if present
    if (!object.keypoints.empty()) {
        to_ros_msg(&output_msg->keypoints, object.keypoints);
    }
}

void to_ros_msg(redoxi_public_msgs::msg::Keypoints *output_msg,
                const std::vector<det_types::Keypoint> &keypoints)
{
    if (!output_msg) {
        return;
    }

    redoxi_public_msgs::msg::Keypoints keypoints_msg;
    for (size_t i = 0; i < keypoints.size(); i++) {
        const auto &kp = keypoints[i];
        geometry_msgs::msg::Point point;
        point.x = kp.xy[0];
        point.y = kp.xy[1];
        keypoints_msg.keypoints_2.push_back(point);
        keypoints_msg.confidence.push_back(kp.score);
        keypoints_msg.semantic_type.push_back(i);
    }
    *output_msg = keypoints_msg;
}

void to_ros_msg(std::vector<redoxi_public_msgs::msg::Detection> *output_msg,
                const det_types::SingleImageOutput &image_detections)
{
    if (!output_msg) {
        return;
    }

    output_msg->clear();
    for (const auto &object : image_detections.objects) {
        redoxi_public_msgs::msg::Detection detection_msg;
        to_ros_msg(&detection_msg, object);
        output_msg->push_back(detection_msg);
    }
}

} // namespace redoxi_works::inference::conversion
