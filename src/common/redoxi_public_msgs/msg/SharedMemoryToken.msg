# This message is used to pass the shared memory token between node
# A shared memory token contains information about how to access the shared memory to get an object
string SHM_SERVICE_V6D="v6d"
string SHM_SERVICE_BOOST="boost"

# used to identify this message
unique_identifier_msgs/UUID x_uid

# the name of the shared memory service, e.g. "v6d", "boost", etc.
string service_name

# the key of the shared memory region, for v6d, it's the socket name, for boost, it's the name of the region
string region_key

# the size of the object in the shared memory region, in bytes
# -1 means the object does not exist
# if object_key or object_id is set, you should also set object_size to non-negative value
# otherwise, the receiver may assume the object does not exist
int64 object_size -1

# the key of the object in the shared memory region, empty means the object is not identified by a key
string object_key

# the id of the object in the shared memory region
int64 object_id 0
