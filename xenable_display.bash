#!/bin/bash

# Get the SSH client IP address from the SSH_CLIENT environment variable
# SSH_CLIENT contains "<client_ip> <client_port> <server_port>"
CLIENT_IP=$(echo $SSH_CLIENT | awk '{print $1}')

# xdisplay number on the client machine
DISPLAY_NUMBER=5.0

# If SSH_CLIENT is empty, try SSH_CONNECTION as fallback
if [ -z "$CLIENT_IP" ]; then
    # SSH_CONNECTION contains "<client_ip> <client_port> <server_ip> <server_port>" 
    CLIENT_IP=$(echo $SSH_CONNECTION | awk '{print $1}')
fi

# Print the client IP if found
if [ -n "$CLIENT_IP" ]; then
    echo "SSH client IP address: $CLIENT_IP"
else
    echo "Could not determine SSH client IP address"
    exit 1
fi

# Check if running in WSL
if grep -qi microsoft /proc/version; then
    echo "Running in WSL, setting DISPLAY to host.docker.internal:$DISPLAY_NUMBER"
    export DISPLAY=host.docker.internal:$DISPLAY_NUMBER

    # opengl use external display
    # echo "setting LIBGL_ALWAYS_INDIRECT=1 to use external opengl gl"
    # export LIBGL_ALWAYS_INDIRECT=1
else
    echo "Running in non-WSL, setting DISPLAY to $CLIENT_IP:$DISPLAY_NUMBER"
    export DISPLAY=$CLIENT_IP:$DISPLAY_NUMBER
fi
echo "DISPLAY set to $DISPLAY"