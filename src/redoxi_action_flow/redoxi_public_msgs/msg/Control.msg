# control signal codes, attached to a message to indicate the special action

# IMPORTANT:
# if a message contains other message that contains control msg
# only the top level control message is used

# this signal means nothing, please ignore it
int64 NORMAL=0

# ping signal, 
# the upstream expects the downstream to reply normally, but do not process the data
int64 PING=1

# flush signal
# the upstream expects the downstream to finish processing the previous data, and process
# the new data in a clean state not affected by previous data, for example, a new segment
# of video frames
int64 FLUSH=2

# reset signal
# after receiving this signal, the upstream assumes downstreams reset to a state
# as if the previous data never exists, like a restart
int64 RESET=3

# terminate signal
# after receiving this signal, the upstream will not send any more data
int64 TERMINATE=4

int64 code 0
string text_msg