# metadata for tensor, mainly used in shared memory

# the shape of the tensor
# empty shape means this tensor metadata itself is invalid
int64[] shape

# the data type of the tensor
# following numpy convention, that is, "float32", "float64", "int32", "int64", etc.
# it must be <type_name><number_of_bits>, where number_of_bits is REQUIRED
# meaning, you cannot use "float" or "int" as the type name
string dtype
