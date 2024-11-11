#ifndef REDOXI_SHARED_MEMORY__REDOXI_SHARED_MEMORY_HPP_
#define REDOXI_SHARED_MEMORY__REDOXI_SHARED_MEMORY_HPP_

#include <redoxi_shared_memory/visibility_control.h>
#include <atomic>
#include <string>
#include <map>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <opencv2/opencv.hpp>

namespace redoxi_works
{

namespace shared_memory
{

struct ObjectIdentifier {
    using RawPtr = ObjectIdentifier *;

    std::optional<int64_t> id = std::nullopt;
    std::optional<std::string> key = std::nullopt;
};

//! Connect params for shared memory client, customize by subclass
struct KeyValueStore {
    virtual ~KeyValueStore() = default;
    using RawPtr = KeyValueStore *;

    //! Get a string value from the connect params
    virtual int get_string(std::string *output, const std::string &key) const = 0;

    //! Set a string value in the connect params
    //! @return 0 if success, -1 if failed
    virtual int set_string(const std::string &key, const std::string &value) = 0;

    //! Get a int64_t value from the connect params
    virtual int get_int(int64_t *output, const std::string &key) const = 0;

    //! Set a int64_t value in the connect params
    //! @return 0 if success, -1 if failed
    virtual int set_int(const std::string &key, int64_t value) = 0;

    //! Get a uint64_t value from the connect params
    virtual int get_uint(uint64_t *output, const std::string &key) const = 0;

    //! Set a uint64_t value in the connect params
    //! @return 0 if success, -1 if failed
    virtual int set_uint(const std::string &key, uint64_t value) = 0;

    //! Get a double value from the connect params
    virtual int get_double(double *output, const std::string &key) const = 0;

    //! Set a double value in the connect params
    //! @return 0 if success, -1 if failed
    virtual int set_double(const std::string &key, double value) = 0;

    //! Check if the connect params has a key
    virtual bool has_key(const std::string &key) const = 0;
};

namespace detail
{
//! Default implementation of ConnectParams, using std::map to store the values
struct DefaultKeyValueStore : public KeyValueStore {
    int get_string(std::string *output, const std::string &key) const override
    {
        auto it = m_name2string.find(key);
        if (it != m_name2string.end()) {
            *output = it->second;
            return 0; //! Return 0 for success
        }
        return -1; //! Return -1 if key not found
    }

    int set_string(const std::string &key, const std::string &value) override
    {
        m_name2string[key] = value;
        return 0; //! Always return 0 for success
    }

    int get_int(int64_t *output, const std::string &key) const override
    {
        auto it = m_name2int.find(key);
        if (it != m_name2int.end()) {
            *output = it->second;
            return 0; //! Return 0 for success
        }
        return -1; //! Return -1 if key not found
    }

    int set_int(const std::string &key, int64_t value) override
    {
        m_name2int[key] = value;
        return 0; //! Always return 0 for success
    }

    int get_uint(uint64_t *output, const std::string &key) const override
    {
        auto it = m_name2uint.find(key);
        if (it != m_name2uint.end()) {
            *output = it->second;
            return 0; //! Return 0 for success
        }
        return -1; //! Return -1 if key not found
    }

    int set_uint(const std::string &key, uint64_t value) override
    {
        m_name2uint[key] = value;
        return 0; //! Always return 0 for success
    }

    int get_double(double *output, const std::string &key) const override
    {
        auto it = m_name2double.find(key);
        if (it != m_name2double.end()) {
            *output = it->second;
            return 0; //! Return 0 for success
        }
        return -1; //! Return -1 if key not found
    }

    int set_double(const std::string &key, double value) override
    {
        m_name2double[key] = value;
        return 0; //! Always return 0 for success
    }

    bool has_key(const std::string &key) const override
    {
        return m_name2string.find(key) != m_name2string.end() ||
               m_name2int.find(key) != m_name2int.end() ||
               m_name2uint.find(key) != m_name2uint.end() ||
               m_name2double.find(key) != m_name2double.end();
    }

  protected:
    std::map<std::string, std::string> m_name2string;
    std::map<std::string, int64_t> m_name2int;
    std::map<std::string, uint64_t> m_name2uint;
    std::map<std::string, double> m_name2double;

    // you must subclass this class to use it
    DefaultKeyValueStore() = default;
};
} // namespace detail

struct DataBlock {
    virtual ~DataBlock() = default;
    using RawPtr = DataBlock *;

    //! construct from bytes reference
    //! @note this is a reference, the data block will not own the data, you must ensure the data is valid during the lifetime of the data block
    virtual void from_bytes_ref(const uint8_t *data, size_t size) = 0;

    //! get as bytes reference, return 0 if success, -1 if failed
    //! If data==nullptr and size==nullptr, it will only check existence of the data
    //! @note this is a reference, if the data block is created by someone else, you must ensure the datablock is alive during the data is used
    virtual int get_as_bytes_ref(uint8_t **data, size_t *size) const = 0;

    //! construct from opencv mat
    virtual void from_cvmat(const cv::Mat &input) = 0;

    //! get as opencv mat, return 0 if success, -1 if failed
    //! If output==nullptr, it will only check existence of the data
    //! @note this is a reference, if the data block is created by someone else, you must ensure the datablock is alive during the data is used
    virtual int get_as_cvmat(cv::Mat *output) const = 0;

    //! check if the data returned by the data block is mutable (writable)
    virtual bool is_data_mutable() const = 0;

    //! has data stored in shared memory, but mapped to this process's address?
    virtual bool has_remote_data() const = 0;

    //! has data from this process, but not yet submitted to shared memory?
    virtual bool has_local_data() const = 0;
};

namespace detail
{
//! Default implementation of DataBlock, using std::variant to store the data
//! @note This is a default implementation, it is intended to be used as a base class
struct DefaultDataBlock : public DataBlock {
    void from_bytes_ref(const uint8_t *data, size_t size) override
    {
        m_data = cv::Mat(size, 1, CV_8UC1, const_cast<uint8_t *>(data));
        m_is_writable = true;
        m_has_local_data = true;
        m_has_remote_data = false;
    }

    int get_as_bytes_ref(uint8_t **data, size_t *size) const override
    {
        if (m_data.has_value()) {
            uint8_t *bytes = m_data.value().data;
            if (size != nullptr) {
                *size = m_data.value().total() * m_data.value().elemSize();
            }
            if (data != nullptr) {
                *data = bytes;
            }
            return 0; //! Return 0 for success
        }
        return -1; //! Return -1 if data is not in byte format
    }

    void from_cvmat(const cv::Mat &input) override
    {
        m_data = input;
        m_is_writable = true;
        m_has_local_data = true;
        m_has_remote_data = false;
    }

    int get_as_cvmat(cv::Mat *output) const override
    {
        if (m_data.has_value()) {
            if (output != nullptr) {
                *output = m_data.value();
            }
            return 0; //! Return 0 for success
        }
        return -1; //! Return -1 if data is not in cv::Mat format
    }

    bool is_data_mutable() const override
    {
        return m_is_writable;
    }

    bool has_remote_data() const override
    {
        return m_has_remote_data;
    }

    bool has_local_data() const override
    {
        return m_has_local_data;
    }

  protected:
    std::optional<cv::Mat> m_data;
    std::atomic<bool> m_is_writable{false};
    std::atomic<bool> m_has_remote_data{false};
    std::atomic<bool> m_has_local_data{false};

    // you must subclass this class to use it
    DefaultDataBlock() = default;
};
} // namespace detail

} // namespace shared_memory

} // namespace redoxi_works

#endif // REDOXI_SHARED_MEMORY__REDOXI_SHARED_MEMORY_HPP_
