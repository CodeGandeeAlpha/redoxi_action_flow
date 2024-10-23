#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_video_reader/generators/SimpleActionGenerator.hpp>

namespace redoxi_works
{

SimpleActionGenerator::SimpleActionGenerator(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase(name, options)
{
}

int SimpleActionGenerator::_read_frame(cv::Mat &frame, std::atomic<int64_t> &frame_number)
{
    //! Randomize the frame
    cv::RNG rng(cv::getTickCount());
    cv::Scalar randomColor(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    frame = cv::Mat(480, 640, CV_8UC3, randomColor);

    //! Add some random shapes
    int numShapes = rng.uniform(1, 5);
    for (int i = 0; i < numShapes; ++i) {
        int shapeType = rng.uniform(0, 3);
        cv::Point pt1(rng.uniform(0, frame.cols), rng.uniform(0, frame.rows));
        cv::Point pt2(rng.uniform(0, frame.cols), rng.uniform(0, frame.rows));
        cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

        if (shapeType == 0) {
            cv::rectangle(frame, pt1, pt2, color, -1);
        } else if (shapeType == 1) {
            cv::circle(frame, pt1, rng.uniform(10, 50), color, -1);
        } else {
            std::vector<cv::Point> pts;
            for (int j = 0; j < 3; ++j) {
                pts.push_back(cv::Point(rng.uniform(0, frame.cols), rng.uniform(0, frame.rows)));
            }
            cv::fillPoly(frame, std::vector<std::vector<cv::Point>>{pts}, color);
        }
    }

    frame_number++;

    return 0;
}

void SimpleActionGenerator::_step()
{
    // if not started yet, do nothing
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    // create a local task group
    // accumulate tasks and execute them in one go
    tbb::task_group unordered_tasks;

    // just print a heartbeat
    // RDX_LOG_INFO(this, __func__, true, "heartbeat");

    // ping all downstreams
    for (const auto &downstream : m_downstreams) {
        RDX_LOG_INFO(this, __func__, true, "pinging downstream: {}", downstream.first);
        auto client = downstream.second->accept_frame;

        Downstream_t::Goal_t goal;
        Downstream_t::SendGoalOptions_t opt;
        opt.goal_response_callback = [this, downstream](const auto &goal_handle) {
            bool downstream_accepted = goal_handle != nullptr;
            RDX_LOG_INFO(this, __func__, true, "received goal response from downstream: {} (accepted: {})",
                         downstream.first, downstream_accepted);
        };
        opt.result_callback = [this, downstream](const auto &result) {
            RDX_LOG_INFO(this, __func__, true, "received result from downstream: {} (status: {}, goal_id: {})",
                         downstream.first, (int)result.code, to_boost_uuid_string(result.goal_id));
        };

        goal.x_control.code = goal.x_control.PING;
        goal.x_uid = to_ros_uuid_msg(boost::uuids::random_generator()());
        client->async_send_goal(goal, opt);
        RDX_LOG_INFO(this, __func__, true, "sent ping to downstream: {}", downstream.first);

        // block myself
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        RDX_LOG_INFO(this, __func__, true, "unblocked myself");
    }

    return;

    // read next frame, if token is ready
    DummyTimeToken token;
    if (m_impl->read_frame_token->try_pop_token(token)) {
        // time to read a new frame
        cv::Mat frame;
        int ret = _read_frame(frame, m_frame_number);
        if (ret == 0) {
            // create a frame delivery task and deliver
            FrameDeliveryTask_t delivery_task;
            _create_frame_delivery_task(frame, delivery_task);

            auto task_func = [this, delivery_task]() {
                const auto &qos = m_runtime_config->frame_delivery_options;

                // at least try once
                bool task_sent = m_impl->frame_delivery_node->put_data(delivery_task);

                // if qos says you should retry until success, do so
                if (qos->drop_frame_strategy == FrameDeliveryOptions_t::DropFrameStrategy::NoDrop) {
                    auto max_attempts = DefaultParams::MaxNumberOfRetries;
                    int attempts = 0;
                    while (!task_sent && attempts < max_attempts) {
                        attempts++;
                        std::this_thread::sleep_for(qos->deliver_retry_interval);
                        task_sent = m_impl->frame_delivery_node->put_data(delivery_task);
                    }

                    // flush the graph, make sure the frame is delivered
                    m_impl->frame_delivery_graph->wait_for_all();
                }

                if (task_sent) {
                    RCLCPP_DEBUG(this->get_logger(), "[%s][_step()] Frame %d (UID: %s) sent",
                                 this->get_name(), (int)delivery_task.frame_number,
                                 delivery_task.get_uid_as_string().c_str());
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "[%s][_step()] Frame %d (UID: %s) dropped",
                                 this->get_name(), (int)delivery_task.frame_number,
                                 delivery_task.get_uid_as_string().c_str());
                }
            };

            // execute the task in isolation
            unordered_tasks.run(task_func);

            // requested publish frame?
            if (m_publish_image) {
                unordered_tasks.run([this, frame]() {
                    _publish_frame(frame);
                });
            }
        }
    }

    // wait for all tasks to complete
    unordered_tasks.wait();
}
} // namespace redoxi_works