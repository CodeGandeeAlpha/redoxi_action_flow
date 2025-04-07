#pragma once

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace redoxi_works
{
//! UUID type for redoxi works
//! Note: UUID when created, is not guaranteed to be all 0, you need to UUIDType uid{0} to make it all 0
using UUIDType = boost::uuids::uuid;

//! Trait for UUID, handling conversion between string, array and UUID type
struct UUIDTrait {
    static UUIDType generate()
    {
        return boost::uuids::random_generator()();
    }

    static std::string to_string(const UUIDType &uuid)
    {
        return boost::uuids::to_string(uuid);
    }

    static std::array<uint8_t, 16> to_array(const UUIDType &uuid)
    {
        std::array<uint8_t, 16> output;
        std::copy(uuid.begin(), uuid.end(), output.begin());
        return output;
    }

    static UUIDType from_string(const std::string &str)
    {
        return boost::uuids::string_generator()(str);
    }

    static UUIDType from_array(const std::array<uint8_t, 16> &arr)
    {
        UUIDType uuid;
        std::copy(arr.begin(), arr.end(), uuid.begin());
        return uuid;
    }
};
} // namespace redoxi_works