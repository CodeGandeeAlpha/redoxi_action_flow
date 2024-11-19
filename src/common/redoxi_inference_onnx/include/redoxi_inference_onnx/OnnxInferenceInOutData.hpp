#pragma once

#include <redoxi_inference_onnx/OnnxPortData.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>
#include <onnxruntime_cxx_api.h>
#include <map>
#include <atomic>

namespace redoxi_works::inference::onnx
{
class OnnxModelInference;

// InOut data for inference
// @note: this class is not thread safe, only use it in a single thread
class OnnxInferenceInOutData : public InferenceInOutData
{
    friend class OnnxModelInference;

  public:
    using Ptr = std::shared_ptr<OnnxInferenceInOutData>;
    using ConstPtr = std::shared_ptr<const OnnxInferenceInOutData>;

  public:
    OnnxInferenceInOutData() = default;
    virtual ~OnnxInferenceInOutData() = default;

    virtual ModelPortInfo::ConstPtr get_input_port_info(const std::string &port_name) const override;
    virtual ModelPortInfo::ConstPtr get_output_port_info(const std::string &port_name) const override;

    virtual ModelPortData::Ptr get_input_port_data(const std::string &port_name) override;
    virtual ModelPortData::Ptr get_output_port_data(const std::string &port_name) override;

    virtual RedoxiModelInference *get_owner() override;
    virtual const RedoxiModelInference *get_owner() const override;

    // initialize the inout data
    void init(OnnxModelInference *model_inference);

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

    OnnxModelInference *m_model_inference = nullptr;

    // io binding for this inout data, including all ports
    std::shared_ptr<Ort::IoBinding> m_io_binding;

    // port data indexed by port name
    std::map<std::string, OnnxPortData::Ptr> m_input_ports;
    std::map<std::string, OnnxPortData::Ptr> m_output_ports;

    // flag to indicate if the port configuration has been changed
    std::atomic_bool m_input_port_configuration_dirty{false};
    std::atomic_bool m_output_port_configuration_dirty{false};

    // whenever possible, use io binding to do inference?
    std::atomic_bool m_use_io_binding{true};

    // in io-binding, should we bind to output tensor whenever possible?
    // if false, we will always bind to memory info instead
    std::atomic_bool m_prefer_bind_output_tensor{true};

    // flag to indicate if the output port is bound by tensor
    // if false, the port is bound by memory info
    std::map<std::string, bool> m_output_port_bound_by_tensor;

    // any data attached to the inference inout data, for custom usage without inheritance
    std::map<std::string, std::shared_ptr<std::any>> m_any_data;
};

} // namespace redoxi_works::inference::onnx
