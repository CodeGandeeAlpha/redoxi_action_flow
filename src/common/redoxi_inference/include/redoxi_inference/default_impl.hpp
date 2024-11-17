#pragma once

#include <redoxi_inference/redoxi_inference.hpp>
#include <any>

namespace redoxi_works::inference::defaults
{

class DefaultKeyValueStore : public KeyValueStore
{
  public:
    DefaultKeyValueStore() = default;
    virtual ~DefaultKeyValueStore() = default;

    const std::vector<KeyValueStore::KeyInfo> &get_all_keys() const override
    {
        return m_all_keys;
    }

    int get_string(std::string *output, const std::string &key) const override
    {
        auto it = m_name_to_variable_addr.find(key);
        if (it != m_name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<std::string *>(it->second);
                if (output && variable_addr) {
                    *output = *variable_addr;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }

    int set_string(const std::string &key, const std::string &value) override
    {
        auto it = m_name_to_variable_addr.find(key);
        if (it != m_name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<std::string *>(it->second);
                if (variable_addr) {
                    *variable_addr = value;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        //! Key not found
        return -1;
    }

    int get_int(int64_t *output, const std::string &key) const override
    {
        auto it = m_name_to_variable_addr.find(key);
        if (it != m_name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<int64_t *>(it->second);
                if (output && variable_addr) {
                    *output = *variable_addr;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }

    int set_int(const std::string &key, int64_t value) override
    {
        auto it = m_name_to_variable_addr.find(key);
        if (it != m_name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<int64_t *>(it->second);
                if (variable_addr) {
                    *variable_addr = value;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }

    int get_double(double *output, const std::string &key) const override
    {
        auto it = m_name_to_variable_addr.find(key);
        if (it != m_name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<double *>(it->second);
                if (output && variable_addr) {
                    *output = *variable_addr;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }

    int set_double(const std::string &key, double value) override
    {
        auto it = m_name_to_variable_addr.find(key);
        if (it != m_name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<double *>(it->second);
                if (variable_addr) {
                    *variable_addr = value;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }

    bool has_key(const std::string &key) const override
    {
        for (const auto &key_info : get_all_keys()) {
            if (key_info.name == key) {
                return true;
            }
        }
        return false;
    }

  protected:
    //! Register a new key with its variable address
    template <typename T>
    void register_key(const KeyValueStore::KeyInfo &key_info, T *variable_addr)
    {
        // this key should be unique
        if (m_name_to_keyinfo.find(key_info.name) != m_name_to_keyinfo.end()) {
            throw std::runtime_error("Key already exists: " + key_info.name);
        }

        auto dtype = key_info.dtype;

        //! Check if type T matches with dtype
        if (dtype == "string" && !std::is_same_v<T, std::string>) {
            throw std::runtime_error("Type mismatch: expected std::string for dtype 'string'");
        } else if (dtype == "int64" && !std::is_same_v<T, int64_t>) {
            throw std::runtime_error("Type mismatch: expected int64_t for dtype 'int64'");
        } else if (dtype == "float64" && !std::is_same_v<T, double>) {
            throw std::runtime_error("Type mismatch: expected double for dtype 'float64'");
        } else if (dtype != "string" && dtype != "int64" && dtype != "float64") {
            throw std::runtime_error("Unsupported dtype: " + dtype);
        }

        m_all_keys.push_back(key_info);
        m_name_to_keyinfo[key_info.name] = key_info;
        m_name_to_variable_addr[key_info.name] = variable_addr;
    }

    std::vector<KeyValueStore::KeyInfo> m_all_keys;
    std::map<std::string, KeyValueStore::KeyInfo> m_name_to_keyinfo;
    std::map<std::string, std::any> m_name_to_variable_addr;
};

// some default implementation for the interface
} // namespace redoxi_works::inference::defaults
