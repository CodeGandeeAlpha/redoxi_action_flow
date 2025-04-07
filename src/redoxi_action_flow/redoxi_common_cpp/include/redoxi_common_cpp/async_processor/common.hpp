#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <cstddef>

namespace redoxi_works
{

namespace async_processor
{

//! Default execute token size for async processor nodes
const int DEFAULT_EXECUTE_TOKEN_SIZE = 4;

//! Dummy token for use with tbb graph
struct DefaultExecToken {
    ~DefaultExecToken() = default;

    //! Error code, 0 means success
    //! This code will be set during the execution of the pipeline
    int error_code = 0;
};

struct DefaultInputDataToken : public DefaultExecToken {
    ~DefaultInputDataToken() = default;

    /*!
     * @brief When preserved order is required, the output data will be sequenced using this number.
     * @note You must increment this number one by one, no jump is allowed, otherwise the system will block.
     * If sequence number is the same, the output will be ignored. It must be set to 0 again when the graph is reset.
     */
    std::size_t sequence_number = 0;

    //! Error code, 0 means success
    //! This code will be set during the execution of the pipeline
    int error_code = 0;
};

struct DummyInputData {
    ~DummyInputData() = default;
};

struct DummyOutputData {
    ~DummyOutputData() = default;
};

} // namespace async_processor

} // namespace redoxi_works
