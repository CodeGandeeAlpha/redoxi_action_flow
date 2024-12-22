#pragma once

#include <redoxi_inference_rknn/RknnPortData.hpp>
#include <redoxi_inference_rknn/RknnPortInfo.hpp>
#include <rknn_api.h>
#include <map>
#include <atomic>

namespace redoxi_works::inference::rknn
{
class RknnModelInference;

// InOut data for inference
// @note: this class is not thread safe, only use it in a single thread
class RknnInferenceInOutData : public InferenceInOutData
{
    friend class RknnModelInference;

  public:
    using Ptr = std::shared_ptr<RknnInferenceInOutData>;
    using ConstPtr = std::shared_ptr<const RknnInferenceInOutData>;

  public:
    RknnInferenceInOutData() = default;
    virtual ~RknnInferenceInOutData() = default;

    virtual ModelPortInfo::ConstPtr get_input_port_info(const std::string &port_name) const override;
    virtual ModelPortInfo::ConstPtr get_output_port_info(const std::string &port_name) const override;

    virtual ModelPortData::Ptr get_input_port_data(const std::string &port_name) override;
    virtual ModelPortData::Ptr get_output_port_data(const std::string &port_name) override;

    virtual RedoxiModelInference *get_owner() override;
    virtual const RedoxiModelInference *get_owner() const override;

    // initialize the inout data
    void init(RknnModelInference *model_inference);

    virtual std::map<std::string, std::shared_ptr<std::any>> get_any_data() const override;
    virtual std::shared_ptr<std::any> get_any_data(const std::string &key) const override;
    virtual void set_any_data(const std::string &key, std::shared_ptr<std::any> value) override;
    virtual bool remove_any_data(const std::string &key) override;

  protected:
    // update the io binding for input ports
    // return true if the io binding is updated, false otherwise
    virtual bool _update_io_binding_input();

    // update the io binding for output ports
    // return true if the io binding is updated, false otherwise
    virtual bool _update_io_binding_output();

    RknnModelInference *m_model_inference = nullptr;

    // port data indexed by port name
    std::map<std::string, RknnPortData::Ptr> m_input_ports;
    std::map<std::string, RknnPortData::Ptr> m_output_ports;

    // flag to indicate if the port configuration has been changed
    std::atomic_bool m_input_port_configuration_dirty{false};
    std::atomic_bool m_output_port_configuration_dirty{false};

    // any data attached to the inference inout data, for custom usage without inheritance
    std::map<std::string, std::shared_ptr<std::any>> m_any_data;
};

} // namespace redoxi_works::inference::rknn
