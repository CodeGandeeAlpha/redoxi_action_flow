# track status of an object

# bitmask values
# value representing unspecified state
int64 NONE=0

# value representing the first frame being created
int64 NEW_BIT=1

# value representing still being tracked
int64 OPEN_BIT=2

# value representing short term lost track, wait for re-association
int64 LOST_BIT=4

# value representing no longer being tracked
int64 CLOSE_BIT=8

# value representing missed in the current frame, not matched to any detection
int64 NO_MATCH_BIT=16

# bitmask of track status
int64 bitmask 0