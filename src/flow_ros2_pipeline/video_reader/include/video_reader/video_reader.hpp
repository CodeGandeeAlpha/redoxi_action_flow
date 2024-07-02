#ifndef VIDEO_READER_HPP_
#define VIDEO_READER_HPP_

#include "rclcpp/rclcpp.hpp"
// #include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include "vineyard/client/client.h"
// #include "my_srv/SendVideoFrame2Det.h"
// #include "my_srv/SendVideoFrame2Pose.h"
// #include "my_srv/SendVideoFrame2Tracker.h"
// #include "my_srv/ImageShmNameService.h"
// #include "my_srv/SendImgShm2Det.h"
// #include "my_srv/SendImgShm2Pose.h"
// #include "my_srv/SendImgShm2Tracker.h"
// #include "my_srv/VideoServiceDownstream.h"
// #include "my_srv/ImageShmWriteService.h"

#include "my_msgs/msg/image1080p.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"


namespace FlowRos2Pipeline{
    class VideoReader : public rclcpp::Node {
        public:
            // virtual void onInit();
            VideoReader();
            void img_read_sub_callback(const std_msgs::msg::Empty &msg);
            // void sendToAll(my_srv::VideoServiceDownstream& det_srv,
            //                 my_srv::VideoServiceDownstream& pose_srv,
            //                 my_srv::VideoServiceDownstream& tracker_srv);
        private:
            rclcpp::Publisher<my_msgs::msg::Image1080p>::SharedPtr publisher_;
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr img_read_subscripter_;
            rclcpp::Logger logger_;
            // ros::NodeHandle m_node_handle;
            // ros::ServiceClient m_send_to_det_client;
            // ros::ServiceClient m_send_to_posedet_client;
            // ros::ServiceClient m_send_to_tracker_client;

            // std::vector<ros::ServiceClient> m_other_clients;

            // ros::ServiceClient m_img_shm_name_client;

            // ros::Subscriber m_write_enable_sub;

            // ros::Publisher m_end_flag_pub;
            // ros::Publisher m_time_stamp_pub;

            // bool m_have_created_img_shm = false;
            // bool m_shm_write_enabled = false;

            // std::string m_img_shm_name;

            cv::VideoCapture m_video_capture;
            int m_frame_number = 1;
            cv::Size m_image_size;
            int m_start_frame_number = 1;
            int m_end_frame_number = -1;

            std::shared_ptr<vineyard::Client> m_v6d_client;

            // ros::Time m_start_time;
            // ros::Duration m_total_time;

            // bi::shared_memory_object m_img_shm;
            // bi::mapped_region m_img_shm_region;
    };
}

#endif