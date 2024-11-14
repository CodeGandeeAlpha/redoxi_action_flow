#pragma once

#include <redoxi_inference_onnx/OnnxPortData.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>
#include <onnxruntime_cxx_api.h>
#include <map>

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

    virtual int configure_input_port(
        const std::string &port_name,
        std::vector<int64_t> shape) override;

    virtual ModelPortInfo::ConstPtr get_input_port_info(const std::string &port_name) const override;
    virtual ModelPortInfo::ConstPtr get_output_port_info(const std::string &port_name) const override;

    virtual ModelPortData::Ptr get_input_port_data(const std::string &port_name) override;
    virtual ModelPortData::Ptr get_output_port_data(const std::string &port_name) override;

    virtual void notify_input_data_update();
    virtual void notify_input_configure_update();

    virtual RedoxiModelInference *get_owner() override;
    virtual const RedoxiModelInference *get_owner() const override;

    // initialize the inout data
    void init(OnnxModelInference *model_inference);

  protected:
    OnnxModelInference *m_model_inference = nullptr;

    // io binding for this inout data, including all ports
    std::shared_ptr<Ort::IoBinding> m_io_binding;

    // port data indexed by port name
    std::map<std::string, OnnxPortData::Ptr> m_input_ports;
    std::map<std::string, OnnxPortData::Ptr> m_output_ports;
};
} // namespace redoxi_works::inference::onnx
