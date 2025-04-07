#include <redoxi_dnn_models/_pch.hpp>

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
        detection_msg.frame_metadata.width = image_detections.source_image_size.width;
        detection_msg.frame_metadata.height = image_detections.source_image_size.height;
        output_msg->push_back(detection_msg);
    }
}

void from_ros_msg(det_types::SingleImageOutput *output_detections,
                  const std::vector<redoxi_public_msgs::msg::Detection> &input_msg)
{
    if (!output_detections) {
        return;
    }

    output_detections->objects.clear();
    for (const auto &detection_msg : input_msg) {
        det_types::DetectedObject object;
        from_ros_msg(&object, detection_msg);
        output_detections->objects.push_back(object);

        auto width = detection_msg.frame_metadata.width;
        auto height = detection_msg.frame_metadata.height;
        output_detections->source_image_size = cv::Size(width, height);
    }
}

void from_ros_msg(det_types::DetectedObject *output_object,
                  const redoxi_public_msgs::msg::Detection &input_msg)
{
    if (!output_object) {
        return;
    }

    output_object->class_id = input_msg.category;
    output_object->class_name = input_msg.category_name;
    output_object->score = input_msg.confidence;
    output_object->xywh[0] = input_msg.bbox.x;
    output_object->xywh[1] = input_msg.bbox.y;
    output_object->xywh[2] = input_msg.bbox.width;
    output_object->xywh[3] = input_msg.bbox.height;

    if (!input_msg.keypoints.keypoints_2.empty()) {
        from_ros_msg(&output_object->keypoints, input_msg.keypoints);
    }
}

void from_ros_msg(std::vector<det_types::Keypoint> *output_keypoints,
                  const redoxi_public_msgs::msg::Keypoints &input_msg)
{
    if (!output_keypoints) {
        return;
    }

    output_keypoints->clear();
    for (size_t i = 0; i < input_msg.keypoints_2.size(); ++i) {
        det_types::Keypoint kp;
        kp.xy[0] = input_msg.keypoints_2[i].x;
        kp.xy[1] = input_msg.keypoints_2[i].y;
        kp.score = input_msg.confidence[i];
        output_keypoints->push_back(kp);
    }
}


//! Convert a ROS message to a DetectedObject and return it
det_types::DetectedObject from_ros_msg(const redoxi_public_msgs::msg::Detection &input_msg)
{
    det_types::DetectedObject output_object;
    from_ros_msg(&output_object, input_msg);
    return output_object;
}

//! Convert a ROS message to a vector of Keypoints and return it
std::vector<det_types::Keypoint> from_ros_msg(const redoxi_public_msgs::msg::Keypoints &input_msg)
{
    std::vector<det_types::Keypoint> output_keypoints;
    from_ros_msg(&output_keypoints, input_msg);
    return output_keypoints;
}

//! Convert a vector of ROS Detection messages to a SingleImageOutput and return it
det_types::SingleImageOutput from_ros_msg(const std::vector<redoxi_public_msgs::msg::Detection> &input_msg)
{
    det_types::SingleImageOutput output_detections;
    from_ros_msg(&output_detections, input_msg);
    return output_detections;
}


} // namespace redoxi_works::inference::conversion
