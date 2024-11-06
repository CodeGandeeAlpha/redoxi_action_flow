#ifndef REDOXI_SHARED_MEMORY__REDOXI_SHARED_MEMORY_HPP_
#define REDOXI_SHARED_MEMORY__REDOXI_SHARED_MEMORY_HPP_

#include <redoxi_shared_memory/visibility_control.h>
#include <string>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <opencv2/opencv.hpp>

namespace redoxi_works
{

namespace shared_memory
{

struct Metadata {
    virtual ~Metadata() = default;
};

struct DataBlock {
    virtual ~DataBlock() = default;

    //! Must support raw bytes
    virtual void from_bytes(const uint8_t *data, size_t size) = 0;

    //! get as bytes
    virtual int get_as_bytes(uint8_t *&data, size_t &size) const = 0;

    //! Must support opencv mat
    virtual void from_cvmat(const cv::Mat &input) = 0;

    //! get as opencv mat
    virtual int get_as_cvmat(cv::Mat &output) const = 0;
};

struct ObjectIdentifier {
    std::optional<std::string> key;
    std::optional<int64_t> id;
};

} // namespace shared_memory

} // namespace redoxi_works

#endif // REDOXI_SHARED_MEMORY__REDOXI_SHARED_MEMORY_HPP_
