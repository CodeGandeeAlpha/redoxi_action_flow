#pragma once

#include <redoxi_video_reader_base/VideoReaderBase.hpp>
#include <redoxi_video_reader_base/VideoReaderBaseTypes.hpp>
#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>

#include <tbb/tbb.h>

namespace redoxi_works
{

namespace ap = async_processor;
namespace mytypes = RedoxiVideoReaderBaseTypes;

class RedoxiVideoReaderImpl
{
  public:
    virtual ~RedoxiVideoReaderImpl() = default;
    RedoxiVideoReaderImpl(RedoxiVideoReaderBase *node)
    {
        this->ros_node = node;
    }

    /**
     * @brief Check if shared memory is supported
     * @return true if supported, false otherwise
     */
    virtual bool is_shared_memory_supported()
    {
        return false;
    }

    /**
     * @brief Initialize shared memory storage
     * @return 0 if success, -1 if failed
     */
    virtual int init_shared_memory_storage()
    {
        RDX_ASSERT_CHECK_TRUE(false, "Shared memory is not supported in this implementation");
        return -1;
    }

    /**
     * @brief Add a cv::Mat to shared memory
     * @param data: the data to add
     * @param shared_memory_id: output the id of the shared memory
     * @return 0 if success, -1 if failed
     */
    virtual int add_to_shared_memory(const cv::Mat &data, uint64_t &shared_memory_id)
    {
        RDX_ASSERT_CHECK_TRUE(false, "Shared memory is not supported in this implementation");

        // just ignore the data
        (void)data;
        (void)shared_memory_id;

        // by default, do nothing, but return as failure
        return -1;
    }

    /**
     * @brief Remove data from shared memory
     * @param shared_memory_id: the id of the shared memory to remove
     * @return 0 if success, -1 if failed
     */
    virtual int remove_from_shared_memory(uint64_t shared_memory_id)
    {
        RDX_ASSERT_CHECK_TRUE(false, "Shared memory is not supported in this implementation");
        (void)shared_memory_id;
        return -1;
    }

    RedoxiVideoReaderBase *ros_node = nullptr;

    // thread for step() function
    std::shared_ptr<std::thread> step_thread;

    // token for reading a new frame every x-milliseconds
    std::shared_ptr<RosTimeToken> read_frame_token;

    // frame delivery node
    using FrameDeliveryNode_t = ap::SingleBufferExecNode<mytypes::FrameDeliveryTask, mytypes::FrameDeliveryTask>;
    std::shared_ptr<FrameDeliveryNode_t> frame_delivery_node;
    std::shared_ptr<tbb::flow::graph> frame_delivery_graph;
    tbb::task_group frame_delivery_tg;
};

} // namespace redoxi_works
