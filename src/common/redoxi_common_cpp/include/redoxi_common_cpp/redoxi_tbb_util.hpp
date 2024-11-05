#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_hash.hpp>

namespace redoxi_works
{

//! Hash function for boost::uuids::uuid to be used with tbb::concurrent_hash_map
struct TbbBoostUuidHash {
    size_t hash(const boost::uuids::uuid &id) const
    {
        return boost::hash<boost::uuids::uuid>{}(id);
    }

    //! Equality comparison for boost::uuids::uuid
    bool equal(const boost::uuids::uuid &id1, const boost::uuids::uuid &id2) const
    {
        return id1 == id2;
    }
};

} // namespace redoxi_works
