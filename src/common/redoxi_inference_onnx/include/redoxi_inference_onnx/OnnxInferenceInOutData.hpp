#pragma once

#include <redoxi_inference_onnx/OnnxPortData.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>
#include <onnxruntime_cxx_api.h>
#include <map>
#include <atomic>

namespace redoxi_works::inference::onnx
{
class OnnxModelInference;
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

  protected:
    //! Do something when the port configuration is updated
    //! for example, re-bind the ports when shape is changed
    virtual void _update_port_configuration();

    OnnxModelInference *m_model_inference = nullptr;

    // io binding for this inout data, including all ports
    std::shared_ptr<Ort::IoBinding> m_io_binding;

    // port data indexed by port name
    std::map<std::string, OnnxPortData::Ptr> m_input_ports;
    std::map<std::string, OnnxPortData::Ptr> m_output_ports;

    // flag to indicate if the port configuration is dirty
    std::atomic_bool m_port_configuration_dirty{false};
};
} // namespace redoxi_works::inference::onnx
