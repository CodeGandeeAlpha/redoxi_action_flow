#pragma once

#include <any>
#include <boost/uuid/uuid_generators.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <psg_private_msgs/action/process_psg_document.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>


namespace redoxi_works
{

namespace async_action_document_output_port
{
using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = psg_private_msgs::action::ProcessPsgDocument;
static_assert(RedoxiActionConcept<DeliveryActionType>, "DeliveryActionType must satisfy RedoxiActionConcept");

namespace Defaults
{
static constexpr int64_t FallbackNumberOfRetry = 3;
static constexpr TimeUnit FallbackWaitTimeBetweenRetry = std::chrono::milliseconds(5);
static constexpr TimeUnit FallbackWaitTimeRetryResponse = std::chrono::milliseconds(1000);
} // namespace Defaults

//! Retry policy type implementing the RetryPolicyConcept
class RetryPolicy : public output_port_types::DefaultRetryPolicy<TimeUnit>
{
  public:
    RetryPolicy()
    {
        fallback_number_of_retry = Defaults::FallbackNumberOfRetry;
        fallback_wait_time_between_retry = Defaults::FallbackWaitTimeBetweenRetry;
        fallback_wait_time_retry_response = Defaults::FallbackWaitTimeRetryResponse;
    }
};

//! Source data type for document output port
//! This type must satisfy the DeliverySourceDataConcept
class DeliverySourceData : public output_port_types::SimpleImageSourceData
{
  public:
    using PSGDocument_t = psg_private_msgs::msg::PsgDocument;
    using VisualizationPublisher_t = image_ports::types::DeliverySourceData::VisualizationPublisher_t;
    using DataPublisher_t = image_ports::types::DeliverySourceData::DataPublisher_t;

    DeliverySourceData()
    {
        static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>, "DeliverySourceData must satisfy DeliverySourceDataConcept");
        m_uuid = boost::uuids::random_generator()();
    }
    virtual ~DeliverySourceData() = default;


    //! Get the image
    virtual const PSGDocument_t &get_document() const
    {
        return m_document;
    }

    //! Set the image
    virtual void set_document(const PSGDocument_t &document)
    {
        m_document = document;
    }

    cv::Scalar get_color(const int id) const
    {
        int idx = id * 3;
        cv::Scalar color((37 * idx) % 255, (17 * idx) % 255, (29 * idx) % 255);
        return color;
    }

    //! create debug image for detection visualization
    void create_debug_image_for_detection_visualization(PubVisualizationMsgType_t &msg) const
    {
        //! 转换raw image到cv::Mat
        image_utils::FrameMediator fm(&m_document.frame_bundle.primary_frame);
        cv::Mat cv_image;
        fm.to_cv_image_copy(cv_image);
        auto encoding = fm.get_encoding();

        //! 为不同类别设置不同颜色
        std::map<int, cv::Scalar> class_colors = {
            {0, cv::Scalar(0, 255, 0)}, // 人-绿色
            {1, cv::Scalar(0, 0, 255)}, // 头-红色
            {2, cv::Scalar(255, 0, 0)}, // 脸-蓝色
        };

        //! 在图像上画bbox
        for (const auto &detection : m_document.detections) {
            //! 获取bbox坐标
            int x = static_cast<int>(detection.bbox.x);
            int y = static_cast<int>(detection.bbox.y);
            int width = static_cast<int>(detection.bbox.width);
            int height = static_cast<int>(detection.bbox.height);

            //! 获取类别对应的颜色
            cv::Scalar color = class_colors[detection.category];

            //! 画框
            cv::rectangle(cv_image,
                          cv::Point(x, y),
                          cv::Point(x + width, y + height),
                          color, 2);

            //! 添加类别标签
            std::string label = std::to_string(detection.category) + " " +
                                std::to_string(detection.confidence).substr(0, 4);
            cv::putText(cv_image, label,
                        cv::Point(x, y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        color, 2);
        }

        //! 转回sensor_msgs/Image
        image_utils::FrameMediator fm_cv_image(cv_image, encoding);
        fm_cv_image.to_image_msg(msg);
    }

    //! create debug image for pose visualization
    void create_debug_image_for_pose_visualization(PubVisualizationMsgType_t &msg) const
    {
        //! 转换raw image到cv::Mat
        cv::Mat cv_image;
        image_utils::FrameMediator fm(&m_document.frame_bundle.primary_frame);
        fm.to_cv_image_copy(cv_image);

        //! 为关键点设置颜色
        cv::Scalar keypoint_color(0, 255, 0); // 绿色
        cv::Scalar line_color(255, 255, 0);   // 黄色

        //! 在图像上画关键点和骨架连接
        for (const auto &detection : m_document.detections) {
            const auto &keypoints = detection.keypoints;

            //! 在访问数组或指针前添加检查
            if (keypoints.keypoints_2.empty() || keypoints.confidence.empty()) {
                continue;
            }

            //! 画出17个关键点
            for (size_t i = 0; i < keypoints.keypoints_2.size(); i++) {
                if (keypoints.confidence[i] > 0.3) { // 只画置信度大于0.3的点
                    //! 记录关键点的位置和置信度
                    cv::circle(cv_image,
                               cv::Point(keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y),
                               3, keypoint_color, -1);
                }
            }

            //! 画出骨架连接
            //! COCO数据集的17个关键点连接对
            const std::vector<std::pair<int, int>> skeleton = {
                {5, 7}, {7, 9}, {6, 8}, {8, 10}, // 手臂
                {11, 13},
                {13, 15},
                {12, 14},
                {14, 16}, // 腿
                {5, 6},
                {5, 11},
                {6, 12},  // 躯干
                {11, 12}, // 臀部
                {1, 2},
                {1, 3},
                {2, 4},
                {3, 5},
                {4, 6} // 头部和肩膀
            };

            for (const auto &bone : skeleton) {
                if (keypoints.confidence[bone.first] > 0.3 && keypoints.confidence[bone.second] > 0.3) {
                    //! 记录骨架连接的起点和终点
                    cv::line(cv_image,
                             cv::Point(keypoints.keypoints_2[bone.first].x, keypoints.keypoints_2[bone.first].y),
                             cv::Point(keypoints.keypoints_2[bone.second].x, keypoints.keypoints_2[bone.second].y),
                             line_color, 2);
                }
            }
        }

        //! 转回sensor_msgs/Image
        image_utils::FrameMediator fm_cv_image(cv_image, "bgr8");
        fm_cv_image.to_image_msg(msg);
    }

    //! create debug image for person visualization
    void create_debug_image_for_person_visualization(PubVisualizationMsgType_t &msg) const
    {
        //! 转换raw image到cv::Mat
        cv::Mat cv_image;
        image_utils::FrameMediator fm(&m_document.frame_bundle.primary_frame);
        fm.to_cv_image_copy(cv_image);

        //! 在图像上画person相关的框和keypoints
        for (const auto &person : m_document.persons) {
            //! 随机生成颜色
            cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);

            //! 获取body bbox坐标
            if (person.true_body.category == 0) {
                int x = static_cast<int>(person.true_body.bbox.x);
                int y = static_cast<int>(person.true_body.bbox.y);
                int width = static_cast<int>(person.true_body.bbox.width);
                int height = static_cast<int>(person.true_body.bbox.height);

                //! 画body bbox
                cv::rectangle(cv_image,
                              cv::Point(x, y),
                              cv::Point(x + width, y + height),
                              color, 2);
            }

            //! 画body keypoints
            const auto &keypoints = person.true_body.keypoints;

            //! 在访问数组或指针前添加检查
            if (!keypoints.keypoints_2.empty() && !keypoints.confidence.empty()) {
                //! 画出17个关键点
                for (size_t i = 0; i < keypoints.keypoints_2.size(); i++) {
                    if (keypoints.confidence[i] > 0.3) { // 只画置信度大于0.3的点
                        //! 记录关键点的位置和置信度
                        cv::circle(cv_image,
                                   cv::Point(keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y),
                                   3, color, -1);
                    }
                }

                //! 画出骨架连接
                //! COCO数据集的17个关键点连接对
                const std::vector<std::pair<int, int>> skeleton = {
                    {5, 7}, {7, 9}, {6, 8}, {8, 10}, // 手臂
                    {11, 13},
                    {13, 15},
                    {12, 14},
                    {14, 16}, // 腿
                    {5, 6},
                    {5, 11},
                    {6, 12},  // 躯干
                    {11, 12}, // 臀部
                    {1, 2},
                    {1, 3},
                    {2, 4},
                    {3, 5},
                    {4, 6} // 头部和肩膀
                };

                for (const auto &bone : skeleton) {
                    if (keypoints.confidence[bone.first] > 0.3 && keypoints.confidence[bone.second] > 0.3) {
                        //! 记录骨架连接的起点和终点
                        cv::line(cv_image,
                                 cv::Point(keypoints.keypoints_2[bone.first].x, keypoints.keypoints_2[bone.first].y),
                                 cv::Point(keypoints.keypoints_2[bone.second].x, keypoints.keypoints_2[bone.second].y),
                                 color, 2);
                    }
                }
            }

            //! 画head bbox
            if (person.true_head.category == 1) {
                int x = static_cast<int>(person.true_head.bbox.x);
                int y = static_cast<int>(person.true_head.bbox.y);
                int width = static_cast<int>(person.true_head.bbox.width);
                int height = static_cast<int>(person.true_head.bbox.height);

                //! 画head bbox
                cv::rectangle(cv_image,
                              cv::Point(x, y),
                              cv::Point(x + width, y + height),
                              color, 2);
            }

            //! 画face bbox
            if (person.true_face.category == 2) {
                int x = static_cast<int>(person.true_face.bbox.x);
                int y = static_cast<int>(person.true_face.bbox.y);
                int width = static_cast<int>(person.true_face.bbox.width);
                int height = static_cast<int>(person.true_face.bbox.height);

                //! 画face bbox
                cv::rectangle(cv_image,
                              cv::Point(x, y),
                              cv::Point(x + width, y + height),
                              color, 2);
            }
        }

        //! 转回sensor_msgs/Image
        image_utils::FrameMediator fm_cv_image(cv_image, "bgr8");
        fm_cv_image.to_image_msg(msg);
    }

    //! create debug image for track visualization
    void create_debug_image_for_track_visualization(PubVisualizationMsgType_t &msg) const
    {
        //! 转换raw image到cv::Mat
        cv::Mat cv_image;
        image_utils::FrameMediator fm(&m_document.frame_bundle.primary_frame);
        fm.to_cv_image_copy(cv_image);

        //! 在图像上画关键点和骨架连接
        for (const auto &person : m_document.persons) {
            //! 根据person的track_id设置颜色
            cv::Scalar color = get_color(person.track_id);

            //! 获取body bbox坐标
            if (person.true_body.category == 0) {
                int x = static_cast<int>(person.true_body.bbox.x);
                int y = static_cast<int>(person.true_body.bbox.y);
                int width = static_cast<int>(person.true_body.bbox.width);
                int height = static_cast<int>(person.true_body.bbox.height);

                //! 画body bbox
                cv::rectangle(cv_image,
                              cv::Point(x, y),
                              cv::Point(x + width, y + height),
                              color, 2);
            }

            //! 画body keypoints
            const auto &keypoints = person.true_body.keypoints;

            //! 在访问数组或指针前添加检查
            if (!keypoints.keypoints_2.empty() && !keypoints.confidence.empty()) {
                //! 画出17个关键点
                for (size_t i = 0; i < keypoints.keypoints_2.size(); i++) {
                    if (keypoints.confidence[i] > 0.3) { // 只画置信度大于0.3的点
                        //! 记录关键点的位置和置信度
                        cv::circle(cv_image,
                                   cv::Point(keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y),
                                   3, color, -1);
                    }
                }

                //! 画出骨架连接
                //! COCO数据集的17个关键点连接对
                const std::vector<std::pair<int, int>> skeleton = {
                    {5, 7}, {7, 9}, {6, 8}, {8, 10}, // 手臂
                    {11, 13},
                    {13, 15},
                    {12, 14},
                    {14, 16}, // 腿
                    {5, 6},
                    {5, 11},
                    {6, 12},  // 躯干
                    {11, 12}, // 臀部
                    {1, 2},
                    {1, 3},
                    {2, 4},
                    {3, 5},
                    {4, 6} // 头部和肩膀
                };

                for (const auto &bone : skeleton) {
                    if (keypoints.confidence[bone.first] > 0.3 && keypoints.confidence[bone.second] > 0.3) {
                        //! 记录骨架连接的起点和终点
                        cv::line(cv_image,
                                 cv::Point(keypoints.keypoints_2[bone.first].x, keypoints.keypoints_2[bone.first].y),
                                 cv::Point(keypoints.keypoints_2[bone.second].x, keypoints.keypoints_2[bone.second].y),
                                 color, 2);
                    }
                }
            }

            //! 画head bbox
            if (person.true_head.category == 1) {
                int x = static_cast<int>(person.true_head.bbox.x);
                int y = static_cast<int>(person.true_head.bbox.y);
                int width = static_cast<int>(person.true_head.bbox.width);
                int height = static_cast<int>(person.true_head.bbox.height);

                //! 画head bbox
                cv::rectangle(cv_image,
                              cv::Point(x, y),
                              cv::Point(x + width, y + height),
                              color, 2);
            }

            //! 画face bbox
            if (person.true_face.category == 2) {
                int x = static_cast<int>(person.true_face.bbox.x);
                int y = static_cast<int>(person.true_face.bbox.y);
                int width = static_cast<int>(person.true_face.bbox.width);
                int height = static_cast<int>(person.true_face.bbox.height);

                //! 画face bbox
                cv::rectangle(cv_image,
                              cv::Point(x, y),
                              cv::Point(x + width, y + height),
                              color, 2);
            }
        }

        //! 转回sensor_msgs/Image
        image_utils::FrameMediator fm_cv_image(cv_image, "bgr8");
        fm_cv_image.to_image_msg(msg);
    }

    //! convert to publish message
    int to_publish_visualization(PubVisualizationMsgType_t &msg) const override
    {
        RDX_INFO_DEV(nullptr, __func__, false, "{}", "调用document output port的to_publish_visualization()");
        auto task_uuid = to_boost_uuid(m_document.x_task_metadata.source_task_uid);
        RDX_INFO_DEV(nullptr, __func__, true, "[task={}] Got goal handle",
                     UUIDTrait::to_string(task_uuid));

        if (auxiliary_data.has_value()) {
            std::string auxiliary_data_str = std::any_cast<std::string>(auxiliary_data);

            if (auxiliary_data_str == "image") {
                RDX_INFO_DEV(nullptr, __func__, false, "{}", "开始绘制图像");
                image_utils::FrameMediator fm(&this->m_document.frame_bundle.primary_frame);
                fm.to_image_msg(msg);
            } else if (auxiliary_data_str == "detection") {
                RDX_INFO_DEV(nullptr, __func__, false, "{}", "开始绘制检测结果");
                create_debug_image_for_detection_visualization(msg);
            } else if (auxiliary_data_str == "pose") {
                RDX_INFO_DEV(nullptr, __func__, false, "{}", "开始绘制姿态结果");
                create_debug_image_for_pose_visualization(msg);
            } else if (auxiliary_data_str == "person") {
                RDX_INFO_DEV(nullptr, __func__, false, "{}", "开始绘制person结果");
                create_debug_image_for_person_visualization(msg);
            } else if (auxiliary_data_str == "track") {
                RDX_INFO_DEV(nullptr, __func__, false, "{}", "开始绘制track结果");
                create_debug_image_for_track_visualization(msg);
            } else {
                return -1;
            }
        }
        return 0;
    }

    int to_publish_data(PubDataMsgType_t &msg) const override
    {
        return to_publish_visualization(msg);
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

  protected:
    // boost::uuids::uuid m_uuid;
    PSGDocument_t m_document;
};


//! Delivery target data type for image output port
using DeliveryTargetDataBase =
    output_port_types::DefaultDeliveryTargetData<DeliveryActionType,
                                                 RedoxiActionDataTrait<DeliveryActionType>,
                                                 sensor_msgs::msg::Image>;
class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    using VisualizationPublisher_t = image_ports::types::DeliveryTargetData::VisualizationPublisher_t;
    DeliveryTargetData()
    {
        static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>, "DeliveryTargetData must satisfy DeliveryTargetDataConcept");
    }
    DeliveryTargetData(const Goal_t &goal)
        : DeliveryTargetDataBase(goal)
    {
    }

    int to_publish_visualization(PubVisualizationMsgType_t &msg) const override
    {
        image_utils::FrameMediator fm(&this->m_goal.document.frame_bundle.primary_frame);
        fm.to_image_msg(msg);
        return 0;
    }

  public:
    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;
};
static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>, "DeliveryTargetData must satisfy DeliveryTargetDataConcept");

//! Stamp data type for image output port (nothing to do here, right now)
using DeliveryStampData = output_port_types::DefaultStampData;

//! Delivery policy type for image output port
using DeliveryPolicy = output_port_types::DefaultDeliveryPolicy<RetryPolicy>;
static_assert(output_port_types::DeliveryPolicyConcept<DeliveryPolicy>, "DeliveryPolicy must satisfy DeliveryPolicyConcept");

//! Request type for image output port
using DeliveryRequestBase = output_port_types::DefaultDeliveryRequest<DeliverySourceData, DeliveryTargetData, DeliveryPolicy, DeliveryStampData>;
class DeliveryRequest : public DeliveryRequestBase
{
  protected:
    virtual int _to_target_data(DeliveryTargetData &target_data) const override
    {
        // apply custom function if set
        if (custom_to_target_data) {
            custom_to_target_data(target_data, *this);
            return 0;
        }

        auto &goal = target_data.get_goal();

        // fill payload
        auto document = this->m_source_data.get_document();

        // convert frame to document
        goal.document = document;

        // // set additional information into the goal
        // using ActionTrait = DeliveryTargetData::ActionDataTrait_t;

        // // set the source data UUID
        // ActionTrait::set_uuid(goal, this->m_source_data.get_uuid());

        // ActionTrait::mark_with_control_signal(goal, get_control_signal_code());

        return 0;
    }

  public:
    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

    // custom function to transform the request to target data, if set, this will override the default behavior
    std::function<void(DeliveryTargetData &target_data, const DeliveryRequest &request)> custom_to_target_data;
};

static_assert(output_port_types::DeliveryRequestConcept<DeliveryRequest>, "DeliveryRequest must satisfy DeliveryRequestConcept");
using DeliveryTask = output_port_types::DefaultDeliveryTask<DeliveryRequest, DeliveryTargetData, RetryPolicy>;
static_assert(output_port_types::DeliveryTaskConcept<DeliveryTask>, "DeliveryTask must satisfy DeliveryTaskConcept");

using Downstream = image_ports::types::DownstreamBaseWithImagePub<DeliveryActionType, DeliveryPolicy>;

//! Downstream spec type for image output port
using DownstreamSpec = Downstream::DownstreamSpec_t;
static_assert(output_port_types::DownstreamSpecConcept<DownstreamSpec>,
              "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");

//! Init config type for image output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec,
                                                        DeliverySourceData::DataPublisher_t,
                                                        DeliveryTargetData::DataPublisher_t>;


//! document output port spec
//! This type must satisfy the AsyncActionOutputPortSpecConcept
//! Any async output port can use this spec as a template argument
struct PSGDocumentOutputPortSpec {
    PSGDocumentOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<PSGDocumentOutputPortSpec>,
                      "PSGDocumentOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
    }
    //! Action type and related types
    using ActionType_t = DeliveryActionType;
    using ActionGoal_t = typename ActionType_t::Goal;
    using ActionResult_t = typename ActionType_t::Result;
    using ActionFeedback_t = typename ActionType_t::Feedback;

    // the action type trait
    using ActionDataTrait_t = RedoxiActionDataTrait<ActionType_t>;

    //! Time unit type
    using TimeUnit_t = TimeUnit;

    //! Retry policy type
    using RetryPolicy_t = RetryPolicy;

    //! Source data type
    using DeliverySourceData_t = DeliverySourceData;

    //! Source data publish message type
    using SourcePubVisualizationMsgType_t = typename DeliverySourceData_t::PubVisualizationMsgType_t;

    //! Source data publisher type
    using SourceVisualizationPublisher_t = DeliverySourceData::VisualizationPublisher_t;

    using SourceDataPublisher_t = DeliverySourceData::DataPublisher_t;
    using SourcePubDataMsgType_t = DeliverySourceData::PubDataMsgType_t;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPubVisualizationMsgType_t = typename DeliveryTargetData_t::PubVisualizationMsgType_t;

    //! Target data publisher type
    using TargetVisualizationPublisher_t = DeliveryTargetData::VisualizationPublisher_t;

    using TargetDataPublisher_t = DeliveryTargetData::DataPublisher_t;
    using TargetPubDataMsgType_t = DeliveryTargetData::PubDataMsgType_t;

    //! Stamp type
    using DeliveryStamp_t = output_port_types::DefaultStampData;

    //! Request type
    using DeliveryRequest_t = DeliveryRequest;

    //! Task type
    using DeliveryTask_t = DeliveryTask;

    //! Delivery policy type
    using DeliveryPolicy_t = DeliveryPolicy;

    //! Downstream spec type
    using DownstreamSpec_t = DownstreamSpec;

    //! Init config type
    using InitConfig_t = InitConfig;

    //! Downstream type
    using Downstream_t = Downstream;
};

} // namespace async_action_document_output_port

} // namespace redoxi_works