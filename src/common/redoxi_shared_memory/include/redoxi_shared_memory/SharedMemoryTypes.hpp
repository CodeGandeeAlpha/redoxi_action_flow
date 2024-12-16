#pragma once

#include <redoxi_shared_memory/visibility_control.h>
#include <string_view>
#include <string>
#include <tuple>
#include <json_struct/json_struct.h>
#include <opencv2/opencv.hpp>

namespace redoxi_works::shared_memory
{
namespace config_keys::node
{
constexpr std::string_view ServiceType = "shm_service_type";
constexpr std::string_view RegionKey = "shm_region_key";
} // namespace config_keys::node

namespace config_keys::env
{
constexpr std::string_view ServiceType = "RDX_SHM_SERVICE_TYPE";
constexpr std::string_view RegionKey = "RDX_SHM_REGION_KEY";
} // namespace config_keys::env

namespace config_values::service_types
{
constexpr std::string_view Vineyard = "vineyard";
} // namespace config_values::service_types

struct SharedMemoryConfig {
    // shm configuration keys and values, just for documentations, do not modify them
    std::string _env_config_service_type = config_keys::env::ServiceType.data();
    std::string _env_config_region_key = config_keys::env::RegionKey.data();
    std::string _node_config_service_type = config_keys::node::ServiceType.data();
    std::string _node_config_region_key = config_keys::node::RegionKey.data();

    // shm service type enum values, just for documentations, do not modify them
    std::string _service_type_vineyard = config_values::service_types::Vineyard.data();

    // shm region key, if not given, then read from env variable
    // if given, ignore env variable
    std::string region_key;

    // service name, if not given, then read from env variable
    // if given, ignore env variable
    std::string service_type;

    //! Comparison operators for use in std::map
    bool operator<(const SharedMemoryConfig &other) const
    {
        return std::tie(service_type, region_key) < std::tie(other.service_type, other.region_key);
    }

    bool operator==(const SharedMemoryConfig &other) const
    {
        return std::tie(service_type, region_key) == std::tie(other.service_type, other.region_key);
    }

    bool operator!=(const SharedMemoryConfig &other) const
    {
        return !(*this == other);
    }

    bool is_valid() const
    {
        return !service_type.empty() && !region_key.empty();
    }

    JS_OBJECT(JS_MEMBER(_env_config_region_key),
              JS_MEMBER(_env_config_service_name),
              JS_MEMBER(_node_config_region_key),
              JS_MEMBER(_node_config_service_name),
              JS_MEMBER(_service_type_vineyard),
              JS_MEMBER(region_key),
              JS_MEMBER(service_type));
};


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

} // namespace redoxi_works::shared_memory