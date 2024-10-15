#ifndef REDOXI_VIDEO_READER_BASE__REDOXI_VIDEO_READER_BASE_HPP_
#define REDOXI_VIDEO_READER_BASE__REDOXI_VIDEO_READER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <redoxi_video_reader_base/visibility_control.h>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_video_reader_base/redoxi_video_reader_types.hpp>

namespace redoxi_works
{

class REDOXI_VIDEO_READER_BASE_PUBLIC
    RedoxiVideoReaderBase : public rclcpp::Node,
                            public IOpenCloseProtocol
{
  public:
    //! Import all names from RedoxiVideoReaderInternalTypes
    //! @note: this is to allow subclass to override the type definitions
    using ACT_AcceptFrame = RedoxiVideoReaderInternalTypes::ACT_AcceptFrame;
    using InitConfig = RedoxiVideoReaderInternalTypes::InitConfig;
    using RuntimeConfig = RedoxiVideoReaderInternalTypes::RuntimeConfig;
    using MSG_Frame = RedoxiVideoReaderInternalTypes::MSG_Frame;
    using GoalHandle = RedoxiVideoReaderInternalTypes::GoalHandle;
    using Downstream = RedoxiVideoReaderInternalTypes::Downstream;

  public:
    //! Constructor with node options and name
    explicit RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    //! Destructor
    virtual ~RedoxiVideoReaderBase();

  public:
    //! debug topic for visualization
    virtual std::string get_debug_topic_name() const
    {
        return "Image";
    }

    //! debug topic for visualization
    virtual int get_debug_topic_queue_size() const
    {
        return 10;
    }

    virtual void set_debug_topic_enable(bool enable)
    {
        m_debug_topic_enable = enable;
    }

  public:
    //! Initialize with configurations, must be called once before open()
    virtual int init(const std::shared_ptr<InitConfig> &config,
                     const std::shared_ptr<RuntimeConfig> &runtime_config);

    //! You can set configuration before open() or after close()
    virtual int update_init_config(const std::shared_ptr<InitConfig> &config);
    virtual const std::shared_ptr<InitConfig> &get_init_config() const;

    //! Modify runtime settings, must be called before start(), after stop() or close()
    virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> &config);
    virtual const std::shared_ptr<RuntimeConfig> &get_runtime_config() const;

    //! Open video source, get ready to read
    virtual int open() override;

    //! After calling this, you cannot modify runtime config
    virtual int start() override;

    //! Call this before you modify runtime config
    virtual int stop() override;

    //! Call this before you want to modify init config
    virtual int close() override;

    //! Can modify init config, runtime config

    //! Get the status code of this node
    virtual int get_status_code() const;

  private:
    bool m_debug_topic_enable = false;
};

} // namespace redoxi_works

#endif // REDOXI_VIDEO_READER_BASE__REDOXI_VIDEO_READER_BASE_HPP_
